"""Ground-plane coordinates. Metric scale requires observed floor landmarks.

Conventions: camera x right, y down, z forward; floor x initial forward,
y left, z up. Position is the camera's floor projection, not axle center.
"""
from dataclasses import dataclass
import numpy as np


def unit(value):
    value=np.asarray(value,dtype=float)
    length=np.linalg.norm(value)
    if not np.isfinite(value).all() or length<1e-8:
        raise ValueError("Invalid direction")
    return value/length


def validate_pose(T):
    T=np.asarray(T,dtype=float)
    if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1]):
        raise ValueError("Invalid camera pose")
    R=T[:3,:3]
    if not np.allclose(R.T@R,np.eye(3),atol=1e-4) or np.linalg.det(R)<.999:
        raise ValueError("Pose rotation is invalid")
    return T


@dataclass
class GroundFrame:
    map_id: int
    origin_world: np.ndarray
    axes_world: np.ndarray
    metres_per_slam_unit: float
    camera_height_m: float
    floor_residual_relative: float

    def project(self,T_world_camera,map_id,tracking_state):
        if map_id!=self.map_id or tracking_state!=2:
            raise ValueError("Lost tracking or independent map: metric pose invalid")
        T=validate_pose(T_world_camera)
        xyz=self.axes_world@(T[:3,3]-self.origin_world)*self.metres_per_slam_unit
        if abs(xyz[2]-self.camera_height_m)>max(.02,self.camera_height_m*.15):
            raise ValueError("Camera height inconsistent with calibrated floor")
        forward=self.axes_world@T[:3,2]
        if np.linalg.norm(forward[:2])<.15:
            raise ValueError("Camera optical axis too vertical for heading")
        return np.array([xyz[0],xyz[1],np.arctan2(forward[1],forward[0])])


def fit_ground_frame(T_world_camera,up_camera,height_m,floor_points_world,map_id):
    """Use ONLY user/segmentation-confirmed floor points from this pose/map.

    Measured height supplies metres, observed SLAM floor distance supplies
    arbitrary units. Passing only height and pose cannot determine scale.
    """
    if not np.isfinite(height_m) or not .02<=height_m<=2:
        raise ValueError("Provide measured lens-center height, 0.02..2 m")
    T=validate_pose(T_world_camera)
    n=unit(T[:3,:3]@unit(up_camera))
    points=np.asarray(floor_points_world,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<30 or not np.isfinite(points).all():
        raise ValueError("At least 30 finite, confirmed floor landmarks required")
    offsets=(points-T[:3,3])@n
    distance=-float(np.median(offsets))
    if distance<=1e-6:
        raise ValueError("Floor must be below the camera")
    inliers=np.abs(offsets+distance)<distance*.05
    if inliers.mean()<.8:
        raise ValueError("Floor anchors disagree with calibrated plane normal")
    forward=T[:3,2]-n*np.dot(T[:3,2],n)
    if np.linalg.norm(forward)<.15:
        raise ValueError("Camera too vertical to define forward")
    x=unit(forward);y=unit(np.cross(n,x))
    axes=np.stack((x,y,n))
    planar=(points[inliers]-T[:3,3])@axes[:2].T
    singular=np.linalg.svd(planar-planar.mean(axis=0),compute_uv=False)
    if singular[1]/np.sqrt(len(planar))<distance*.15:
        raise ValueError("Floor anchors need two-dimensional spatial coverage")
    residual=float(np.median(np.abs(offsets[inliers]+distance))/distance)
    return GroundFrame(int(map_id),T[:3,3]-n*distance,axes,height_m/distance,height_m,residual)


def ground_intersections(pixels,K,up_camera,height_m,max_range_m=2.):
    """Intersect UNDISTORTED pixel rays with local floor; return camera XYZ + mask.

    Rejected rays are NaN. Range is horizontal distance from camera projection.
    """
    if height_m<=0 or max_range_m<=0:
        raise ValueError("Height and trusted range must be positive")
    uv=np.asarray(pixels,dtype=float)
    rays=np.column_stack((uv,np.ones(len(uv))))@np.linalg.inv(np.asarray(K)).T
    up=unit(up_camera);denom=rays@up
    valid=np.isfinite(rays).all(axis=1)&(denom<-.05)
    xyz=np.full(rays.shape,np.nan)
    xyz[valid]=rays[valid]*(-height_m/denom[valid,None])
    horizontal=xyz+height_m*up
    valid &= np.linalg.norm(horizontal,axis=1)<=max_range_m
    xyz[~valid]=np.nan
    return xyz,valid


def distance_error(start,end,measured_m):
    if measured_m<=0:
        raise ValueError("Independent measured distance must be positive")
    predicted=float(np.linalg.norm(np.asarray(end)[:2]-np.asarray(start)[:2]))
    return {"measured_m":measured_m,"predicted_m":predicted,
            "signed_error_m":predicted-measured_m,
            "absolute_error_pct":100*abs(predicted-measured_m)/measured_m}
