"""Capture an unmodified Pi JPEG and report brightness and a user-selected neutral ROI."""
import argparse
import json
from pathlib import Path
import sys
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stream import PiStream


def measure(image, roi=None):
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Expected BGR image")
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    result={"width":image.shape[1],"height":image.shape[0],
            "mean_gray":float(gray.mean()),
            "dark_below_30_pct":float((gray<30).mean()*100),
            "clipped_any_channel_pct":float((image.max(axis=2)>=250).mean()*100),
            "mean_rgb":image.mean(axis=(0,1))[::-1].tolist()}
    if roi is not None:
        x,y,w,h=roi
        if min(x,y)<0 or min(w,h)<=0 or x+w>image.shape[1] or y+h>image.shape[0]:
            raise ValueError("Neutral patch must be inside image")
        patch=image[y:y+h,x:x+w]
        valid=(patch.min(axis=2)>20)&(patch.max(axis=2)<245)
        if valid.mean()<.8:
            raise ValueError("Neutral patch is too dark or clipped; move it out of glare")
        rgb=np.median(patch[valid],axis=0)[::-1]
        result.update(neutral_roi=list(roi),neutral_rgb_median=rgb.tolist(),
                      neutral_channel_spread_pct=float((rgb.max()-rgb.min())/rgb.mean()*100),
                      neutral_red_to_green=float(rgb[0]/rgb[1]),
                      neutral_blue_to_green=float(rgb[2]/rgb[1]))
    return result


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output",type=Path,help="New output directory; never overwrites a capture")
    parser.add_argument("--roi",type=int,nargs=4,metavar=("X","Y","WIDTH","HEIGHT"))
    parser.add_argument("--input",type=Path,help="Measure existing image instead of receiving a frame")
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    if args.input:
        jpeg=args.input.read_bytes();image=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
        metadata={"source":str(args.input)}
    else:
        with PiStream() as stream:
            seq,stamp,image,jpeg,_=stream.read()
        metadata={"sequence":seq,"capture_timestamp_ns":stamp}
    (args.output/"frame.jpg").write_bytes(jpeg)
    result=dict(metadata,**measure(image,args.roi))
    (args.output/"quality.json").write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps(result,indent=2))
