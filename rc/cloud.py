"""Bounded voxel downsampling, not a free-space or collision-safety map."""
import numpy as np


class Cloud:
    def __init__(self, voxel_size, capacity=300000):
        if not np.isfinite(voxel_size) or voxel_size <= 0 or capacity < 1:
            raise ValueError("Positive voxel size and capacity required")
        self.voxel_size, self.capacity, self.cells = voxel_size, capacity, {}
        self.dropped = 0

    def add(self, points, colors):
        if not len(points):
            return
        points, colors = np.asarray(points), np.asarray(colors)
        if points.shape != colors.shape or points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("Matching Nx3 points and colors required")
        finite = np.isfinite(points).all(axis=1) & np.isfinite(colors).all(axis=1)
        points, colors = points[finite], colors[finite]
        keys, inverse, count = np.unique(np.floor(points/self.voxel_size).astype(np.int64),
                                       axis=0, return_inverse=True, return_counts=True)
        sums = np.column_stack([np.bincount(inverse, weights=values[:, i], minlength=len(keys))
            for values in (points, colors) for i in range(3)])
        for key, total, n in zip(keys, sums, count):
            key = tuple(key.tolist())
            if key in self.cells:
                old_total, old_n, support = self.cells[key]
                self.cells[key] = old_total+total, old_n+n, support+1
            elif len(self.cells) < self.capacity:
                self.cells[key] = total, n, 1
            else:
                self.dropped += int(n)

    def arrays(self, min_support=1):
        retained = [(total, n) for total, n, support in self.cells.values() if support >= min_support]
        if not retained:
            return np.empty((0, 3)), np.empty((0, 3), np.uint8)
        average = np.array([total/n for total, n in retained])
        return average[:, :3], np.clip(np.rint(average[:, 3:]), 0, 255).astype(np.uint8)


def write_ply(path, points, colors):
    with path.open("w") as output:
        output.write(f"ply\nformat ascii 1.0\ncomment relative monocular scale; not a collision map\n"
                     f"element vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\n"
                     "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
        for p, c in zip(points, colors):
            output.write(" ".join(map(str, [*p, *c]))+"\n")
