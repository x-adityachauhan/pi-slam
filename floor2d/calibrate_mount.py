"""Estimate floor normal from a checkerboard placed FLAT on the floor.

Height must be measured from lens center. Does not establish SLAM scale alone.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def solve_mount(image,intrinsics,height_m,square_mm):
    if not .02<=height_m<=2 or square_mm<=0:
        raise ValueError("Measured positive height and printed square size required")
    if image is None or image.shape[:2]!=(intrinsics["height"],intrinsics["width"]):
        raise ValueError("Image must match calibrated resolution and crop")
    pattern=tuple(intrinsics["pattern"])
    found,corners=cv2.findChessboardCornersSB(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),pattern,
                                            cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        raise ValueError("Complete floor checkerboard not detected")
    obj=np.zeros((pattern[0]*pattern[1],3),np.float64)
    obj[:,:2]=np.mgrid[:pattern[0],:pattern[1]].T.reshape(-1,2)*square_mm/1000
    K=np.array(intrinsics["K"]);D=np.array(intrinsics["distortion"])
    ok,rvec,tvec=cv2.solvePnP(obj,corners,K,D)
    if not ok: raise ValueError("Board pose failed")
    R,_=cv2.Rodrigues(rvec);normal=R[:,2]
    if normal@tvec.ravel()>0: normal=-normal
    inferred_height=-float(normal@tvec.ravel())
    projected,_=cv2.projectPoints(obj,rvec,tvec,K,D)
    rms=float(np.sqrt(np.mean(np.sum((projected-corners)**2,axis=2))))
    if rms>1 or abs(inferred_height-height_m)>max(.015,height_m*.1):
        raise ValueError(f"Mount check failed: reprojection {rms:.3f}px, board height {inferred_height:.3f}m, measured {height_m:.3f}m")
    if np.any((obj@R.T+tvec.ravel())[:,2]<=0):
        raise ValueError("Board behind camera")
    return {"height_m":height_m,"up_camera":normal.tolist(),
            "tilt_down_deg":float(np.degrees(np.arcsin(np.clip(-normal[2],-1,1)))),
            "board_height_m":inferred_height,"reprojection_rms_px":rms,
            "width":intrinsics["width"],"height":intrinsics["height"],
            "square_mm_measured":square_mm,"slam_scale_validated":False,
            "position_reference":"camera projection; axle offset not calibrated"}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("image",type=Path)
    p.add_argument("--intrinsics",type=Path,default=Path("calibration/intrinsics.json"))
    p.add_argument("--height-cm",type=float,required=True)
    p.add_argument("--square-mm",type=float,required=True,help="Actual measured printed square size")
    p.add_argument("--output",type=Path,required=True)
    args=p.parse_args()
    result=solve_mount(cv2.imread(str(args.image)),json.loads(args.intrinsics.read_text()),args.height_cm/100,args.square_mm)
    with args.output.open("x") as f: json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))
