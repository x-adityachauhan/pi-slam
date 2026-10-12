"""Bounded CPU stereo with pose checks and independent-view depth validation.

Depth is validated in image space before voxel downsampling, so agreement is
not determined by which side of an arbitrary voxel boundary a sample falls on.
"""
import json
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import cv2
import numpy as np
from rc.geometry import reconstruct_pair
from rc.quality import ImageCache, epipolar_quality
from rc.cloud import write_ply


def project(points, T, K, distortion):
    camera = (points-T[:3, 3]) @ T[:3, :3]
    # projectPoints also allocates an unused, large Nx15 Jacobian. The common
    # 4/5/8-coefficient pinhole model can be projected directly, without it.
    coefficients = np.asarray(distortion).reshape(-1)
    if len(coefficients) not in (4, 5, 8):
        uv, _ = cv2.projectPoints(camera, np.zeros(3), np.zeros(3), K, distortion)
        return uv.reshape(-1, 2), camera[:, 2]
    d = np.pad(coefficients, (0, 8-len(coefficients)))
    z = np.where(np.abs(camera[:, 2]) > 1e-12, camera[:, 2], 1e-12)
    x, y = camera[:, 0]/z, camera[:, 1]/z
    r2 = x*x+y*y
    radial = (1+d[0]*r2+d[1]*r2*r2+d[4]*r2*r2*r2)/(1+d[5]*r2+d[6]*r2*r2+d[7]*r2*r2*r2)
    xd = x*radial+2*d[2]*x*y+d[3]*(r2+2*x*x)
    yd = y*radial+d[2]*(r2+2*y*y)+2*d[3]*x*y
    return np.column_stack((K[0, 0]*xd+K[0, 2], K[1, 1]*yd+K[1, 2])), camera[:, 2]


def depth_image(points, T, K, distortion, shape):
    """Z-buffer, retaining the closest sample where projections overlap."""
    h, w = shape
    uv, z = project(points, T, K, distortion)
    good = np.isfinite(uv).all(axis=1) & np.isfinite(z) & (z > 0)
    uv, z = uv[good], z[good]
    xy = np.rint(np.clip(uv, -1e7, 1e7)).astype(np.int32)
    good = (xy[:, 0] >= 0) & (xy[:, 0] < w) & (xy[:, 1] >= 0) & (xy[:, 1] < h)
    depth = np.full(h*w, np.inf, np.float32)
    np.minimum.at(depth, xy[good, 1]*w+xy[good, 0], z[good])
    depth[~np.isfinite(depth)] = 0
    return depth.reshape(h, w)


def camera_rays(K, distortion, shape):
    yy, xx = np.indices(shape)
    uv = np.stack((xx, yy), axis=-1).astype(np.float32)
    xy = cv2.undistortPoints(uv.reshape(-1, 1, 2), K, distortion).reshape(*shape, 2)
    return np.concatenate((xy, np.ones((*shape, 1), np.float32)), axis=2)


def consistent_depth(depth, other, T, other_T, K, distortion, rays,
                     relative_tolerance=.01, pixel_tolerance=1.):
    """Return independent-view support and a depth estimate in the first view.

    Both relative depth and the round-trip pixel position must agree. Invalid
    pixels and occlusion boundaries are not interpolated into new surfaces.
    """
    h, w = depth.shape
    yy, xx = np.nonzero(depth > 0)
    support = np.zeros(depth.shape, bool)
    result = np.zeros(depth.shape, np.float32)
    if not len(xx):
        return support, result
    camera = rays[yy, xx]*depth[yy, xx, None]
    world = camera @ T[:3, :3].T+T[:3, 3]
    uv, expected = project(world, other_T, K, distortion)
    xy = np.rint(np.clip(uv, -1e7, 1e7)).astype(np.int32)
    valid = np.isfinite(uv).all(axis=1) & (expected > 0) & (xy[:, 0] >= 0) & (xy[:, 0] < w) & (xy[:, 1] >= 0) & (xy[:, 1] < h)
    ids = np.flatnonzero(valid)
    if not len(ids):
        return support, result
    u, v = xy[ids, 0], xy[ids, 1]
    measured = other[v, u]
    agree = (measured > 0) & (np.abs(measured-expected[ids]) <= relative_tolerance*expected[ids])
    ids, u, v, measured = ids[agree], u[agree], v[agree], measured[agree]
    if not len(ids):
        return support, result
    other_world = (rays[v, u]*measured[:, None]) @ other_T[:3, :3].T+other_T[:3, 3]
    returned, returned_depth = project(other_world, T, K, distortion)
    error = np.linalg.norm(returned-np.column_stack((xx[ids], yy[ids])), axis=1)
    agree = (error <= pixel_tolerance) & (returned_depth > 0)
    support[yy[ids[agree]], xx[ids[agree]]] = True
    result[yy[ids[agree]], xx[ids[agree]]] = returned_depth[agree]
    return support, result


def choose_pairs(views, depth, cache, K, budget):
    candidates = [[] for _ in views]
    for i, first in enumerate(views):
        for j in range(i+1, len(views)):
            second = views[j]
            delta = first["T"][:3, :3].T @ (second["T"][:3, 3]-first["T"][:3, 3])
            baseline = np.linalg.norm(delta)/depth
            angle = np.arccos(np.clip((np.trace(first["T"][:3, :3].T @ second["T"][:3, :3])-1)/2, -1, 1))
            lateral = np.linalg.norm(delta[:2])/max(np.linalg.norm(delta), 1e-9)
            if not (.03 <= baseline <= .25 and angle <= np.deg2rad(35) and lateral >= .45):
                continue
            score = abs(np.log(baseline/.10))+.8*angle+2*(1-lateral)
            candidates[i].append((score, j))
            candidates[j].append((score, i))
    candidates = [sorted(row)[:6] for row in candidates]
    checked, selected, diagnostics = set(), [], []
    # Round-robin coverage avoids spending the entire budget on early frames.
    for rank in range(6):
        for i, row in enumerate(candidates):
            if rank >= len(row):
                continue
            score, j = row[rank]
            pair = tuple(sorted((i, j)))
            if pair in checked:
                continue
            checked.add(pair)
            first, second = cache.get(views[i]["image"]), cache.get(views[j]["image"])
            q = epipolar_quality(first, second, K, views[i]["T"], views[j]["T"])
            accepted = (q["inliers"] >= 18 and q["inlier_fraction"] >= .65
                        and q["median_error_px"] is not None and q["median_error_px"] <= 1.5
                        and min(first["sharpness"], second["sharpness"]) >= 20)
            diagnostics.append(dict(i=i, j=j, accepted=accepted, **q))
            if accepted:
                selected.append(pair)
                if len(selected) >= budget:
                    return selected, diagnostics
    return selected, diagnostics


def fuse_points(points, colors, voxel_size, capacity=300000, return_index=False):
    """Vectorized final downsampling; support was already checked in image space."""
    if not len(points):
        result = (points.reshape(-1, 3), colors.reshape(-1, 3), 0)
        return (*result, np.empty(0, np.int32)) if return_index else result
    keys, inverse, counts = np.unique(np.floor(points/voxel_size).astype(np.int64),
                                      axis=0, return_inverse=True, return_counts=True)
    sums = np.column_stack([np.bincount(inverse, weights=a[:, k], minlength=len(keys))
                            for a in (points, colors) for k in range(3)])
    average = sums/counts[:, None]
    keep = np.argsort(-counts, kind="stable")[:capacity] if len(keys) > capacity else np.arange(len(keys))
    result = (average[keep, :3], np.clip(np.rint(average[keep, 3:]), 0, 255).astype(np.uint8), max(0, len(keys)-capacity))
    if return_index:
        remap = np.full(len(keys), -1, np.int32)
        remap[keep] = np.arange(len(keep))
        return (*result, remap[inverse])
    return result


def depth_triangles(depth, valid, step=2):
    """Connect only measured neighboring pixels; never span depth holes/edges."""
    index = np.full(depth.shape, -1, np.int32)
    index[valid] = np.arange(valid.sum())
    grid = index[::step, ::step]
    a, b, c, d = grid[:-1, :-1], grid[:-1, 1:], grid[1:, :-1], grid[1:, 1:]
    faces = np.concatenate((np.stack((a, b, c), -1).reshape(-1, 3),
                            np.stack((b, d, c), -1).reshape(-1, 3)))
    # Every intervening pixel must also be measured, not just the corners.
    kernel = np.ones((step+1, step+1), np.uint8)
    cells = cv2.erode(valid.astype(np.uint8), kernel, anchor=(0, 0),
                     borderType=cv2.BORDER_CONSTANT, borderValue=0)[::step, ::step][:-1, :-1].ravel()
    faces = faces[np.tile(cells.astype(bool), 2)]
    faces = faces[(faces >= 0).all(axis=1)]
    values = depth[valid][faces]
    return faces[(values.max(axis=1)-values.min(axis=1)) <= .015*values.min(axis=1)]


def write_mesh(path, points, colors, faces):
    with path.open("w") as output:
        output.write(f"ply\nformat ascii 1.0\ncomment measured neighboring depth pixels; holes preserved\n"
                     f"element vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\n"
                     "property uchar red\nproperty uchar green\nproperty uchar blue\n"
                     f"element face {len(faces)}\nproperty list uchar int vertex_indices\nend_header\n")
        np.savetxt(output, np.column_stack((points, colors)), fmt="%.7g %.7g %.7g %d %d %d")
        np.savetxt(output, np.column_stack((np.full(len(faces), 3), faces)), fmt="%d")


def reconstruct_multiview(run, calibration_path, max_pairs=40, voxel_size=None,
                          disparities=128, min_support=2, output_name="dense_v2"):
    from rc.dense_reconstruct import load_views, scene_depth, read_ply_xyz
    run, calibration_path = Path(run), Path(calibration_path)
    if not 1 <= max_pairs <= 120 or min_support < 2:
        raise ValueError("Pair budget must be 1..120 and min_support >= 2")
    if disparities <= 0 or disparities % 16:
        raise ValueError("Disparities must be a positive multiple of 16")
    if not output_name or Path(output_name).name != output_name or output_name in (".", ".."):
        raise ValueError("Output name must be a direct child directory of the run")
    if not (run/output_name).resolve().is_relative_to(run.resolve()):
        raise ValueError("Output directory escaped run")
    calibration = json.loads(calibration_path.read_text())
    if calibration.get("validated") is not True:
        raise ValueError("Validated intrinsics required")
    K, distortion = np.array(calibration["K"], float), np.array(calibration["distortion"], float)
    shape = calibration["height"], calibration["width"]
    files = sorted(run.glob("map_*_keyframes.tum"))
    if not files and (run/"trajectory.tum").exists():
        files = [run/"trajectory.tum"]
    if not files:
        raise ValueError("No optimized poses: SLAM must initialize and retain a map first")
    destination = run/output_name
    destination.mkdir(exist_ok=True)
    rays = camera_rays(K, distortion, shape)
    report = {"metric_scale": False, "flight_ready": False, "vehicle_control": False,
              "method": "pose-checked SGBM, independent-view depth consistency, vectorized voxel fusion",
              "poses": "final optimized per-map poses", "maps": []}
    for poses_file in files:
        started = time.monotonic()
        name = poses_file.name.replace("_keyframes.tum", "").replace("trajectory.tum", "map")
        entry = {"map": name, "points": 0, "attempted_pairs": 0, "successful_pairs": 0,
                 "rejected": [], "independent_coordinate_system": True}
        report["maps"].append(entry)
        try:
            views = load_views(run, poses_file)
            # Bound image cache/depth storage on long DPVO trajectories.
            if len(views) > 80:
                views = [views[i] for i in np.linspace(0, len(views)-1, 80).astype(int)]
            if len(views) < 3:
                raise ValueError("At least three optimized views are required")
            depth = scene_depth(read_ply_xyz(run/(name+".ply")), views[0]["T"])
            voxel = voxel_size if voxel_size is not None else depth/200
            if not np.isfinite(voxel) or voxel <= 0:
                raise ValueError("Positive finite voxel size required")
            cache = ImageCache(K, distortion)
            pairs, entry["pair_checks"] = choose_pairs(views, depth, cache, K, max_pairs)
            entry["voxel_size_map_units"] = voxel
            entry["views"] = len(views)
            # Each reference stores a few independent pair depth estimates.
            # Float32 depth planes are bounded by 2 * max_pairs * H * W.
            estimates = [[] for _ in views]
            partners = [set() for _ in views]
            images_by_view = {i: cache.get(views[i]["image"])["image"] for pair in pairs for i in pair}
            def stereo(pair):
                i, j = pair
                images = [images_by_view[k] for k in (i, j)]
                if any(im.shape[:2] != shape for im in images):
                    raise ValueError("Image dimensions differ from calibration")
                baseline = np.linalg.norm(views[i]["T"][:3, 3]-views[j]["T"][:3, 3])
                count = min(disparities, max(64, int(np.ceil(3*K[0, 0]*baseline/depth/16))*16))
                points, _, _ = reconstruct_pair(*images, K, distortion,
                    views[i]["T"], views[j]["T"], disparities=count, sample_step=1,
                    depth_limit=depth*6, photometric_min=.8, min_disparity_abs=4., alpha=.25,
                    lr_tolerance=.75)
                if len(points) < 100:
                    raise ValueError("Insufficient consistent textured correspondences")
                return points
            def jobs():
                # At most two full stereo results are in flight; do not queue
                # the entire recording's point arrays in the worker pool.
                with ThreadPoolExecutor(max_workers=2) as workers:
                    for start in range(0, len(pairs), 2):
                        pending = [(pair, workers.submit(stereo, pair)) for pair in pairs[start:start+2]]
                        yield from pending
            for (i, j), future in jobs():
                entry["attempted_pairs"] += 1
                try:
                    points = future.result()
                    for a, b in ((i, j), (j, i)):
                        estimates[a].append(depth_image(points, views[a]["T"], K, distortion, shape))
                        partners[a].add(b)
                    entry["successful_pairs"] += 1
                except (ValueError, cv2.error) as exc:
                    entry["rejected"].append(str(exc))
                print(f"{name}: stereo {entry['attempted_pairs']}/{len(pairs)}", flush=True)
            depths = []
            for planes in estimates:
                if not planes:
                    depths.append(np.zeros(shape, np.float32))
                    continue
                stack = np.stack(planes)
                stack[stack == 0] = np.inf
                # Explicit finite median avoids NumPy emitting a warning for
                # every empty pixel (millions of Python calls per recording).
                count_finite = np.isfinite(stack).sum(axis=0)
                ordered = np.sort(stack, axis=0)
                lower = np.maximum((count_finite-1)//2, 0)
                upper = np.maximum(count_finite//2, 0)
                median = .5*(np.take_along_axis(ordered, lower[None], axis=0)[0]
                              + np.take_along_axis(ordered, upper[None], axis=0)[0])
                median[count_finite == 0] = 0
                # Do not average incompatible surfaces across occlusions.
                agreeing = np.abs(stack-median[None]) <= .02*median[None]
                total = np.sum(np.where(agreeing, stack, 0), axis=0)
                count = agreeing.sum(axis=0)
                depths.append(np.divide(total, count, out=np.zeros(shape, np.float32), where=count > 0))
                planes.clear()
            all_points, all_colors, all_faces = [], [], []
            offset = 0
            entry["validated_pixels"] = 0
            entry["depth_pixels"] = sum(int(np.count_nonzero(d)) for d in depths)
            entry["validation_views"] = []
            for i, reference in enumerate(depths):
                if not np.any(reference):
                    continue
                # A reference needs depth from two distinct stereo partners.
                # This prevents a single pair from validating itself backwards.
                if len(partners[i]) < 2:
                    continue
                neighbors = sorted(partners[i], key=lambda j: abs(views[i]["time"]-views[j]["time"]))[:6]
                support = np.zeros(shape, np.uint8)
                accum = reference.copy()
                checks = 0
                for j in neighbors:
                    # Use only depth estimates from another camera with its own
                    # additional partner: at least three distinct images total.
                    if len(partners[j]-{i}) < 1:
                        continue
                    mask, measured = consistent_depth(reference, depths[j], views[i]["T"],
                        views[j]["T"], K, distortion, rays)
                    support += mask.astype(np.uint8)
                    accum += measured
                    checks += 1
                valid = (reference > 0) & (support >= min_support)
                refined = accum/(1+support)
                entry["validated_pixels"] += int(valid.sum())
                entry["validation_views"].append({"view": i, "checks": checks, "pixels": int(valid.sum())})
                camera = rays[valid]*refined[valid, None]
                all_faces.append(depth_triangles(refined, valid)+offset)
                offset += len(camera)
                all_points.append(camera @ views[i]["T"][:3, :3].T+views[i]["T"][:3, 3])
                all_colors.append(cache.get(views[i]["image"])["image"][valid][:, ::-1])
            if not all_points or not sum(map(len, all_points)):
                raise ValueError("No independently consistent depth: record more overlapping translated views")
            points, colors, dropped, remap = fuse_points(np.concatenate(all_points), np.concatenate(all_colors), voxel,
                                                         return_index=True)
            faces = remap[np.concatenate(all_faces)]
            faces = faces[(faces >= 0).all(axis=1)]
            faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 0] != faces[:, 2]) & (faces[:, 1] != faces[:, 2])]
            faces = np.unique(np.sort(faces, axis=1), axis=0)
            if len(faces):
                mesh_file = destination/(name+"_mesh.ply")
                write_mesh(mesh_file, points, colors, faces)
                entry.update(mesh_file=str(mesh_file.relative_to(run)), triangles=len(faces))
            best = max(entry["validation_views"], key=lambda item: item["pixels"])["view"]
            entry["reference_view"] = {"image": str(views[best]["image"].relative_to(run.resolve())),
                "time_s": views[best]["time"], "T_world_camera": views[best]["T"].tolist()}
            output = destination/(name+"_dense.ply")
            write_ply(output, points, colors)
            entry.update(points=len(points), file=str(output.relative_to(run)),
                         capacity_dropped_voxels=dropped, image_reads=cache.reads,
                         min_consistent_other_views=min_support)
        except (ValueError, OSError, cv2.error) as exc:
            entry["error"] = str(exc)
        entry["reconstruction_seconds"] = time.monotonic()-started
    (destination/"stats.json").write_text(json.dumps(report, indent=2))
    if not any(e["points"] for e in report["maps"]):
        raise ValueError("No map passed independent-view depth checks; see reconstruction report")
    return report
