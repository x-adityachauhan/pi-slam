#!/usr/bin/env python3
"""Known-camera procedural 3D scene for an integration smoke test, not calibration.

Uses a separate port. Never use its settings with the physical Pi camera.
"""
import argparse
import socket
import struct
import sys
import time
from pathlib import Path

import cv2
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from calibrate import settings_text


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--folder", type=Path, required=True)
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--frames", type=int, default=240)
    args=parser.parse_args()
    args.folder.mkdir(parents=True,exist_ok=True)
    rng=np.random.default_rng(17)
    points=np.column_stack((rng.uniform(-2.7,2.7,220),rng.uniform(-1.9,1.9,220),rng.uniform(3.2,7,220)))
    textures=[cv2.resize(rng.integers(0,2,(8,8),dtype=np.uint8)*255,(48,48),interpolation=cv2.INTER_NEAREST) for _ in points]
    textures=[cv2.cvtColor(t,cv2.COLOR_GRAY2BGR) for t in textures]
    K=np.array([[520.,0,320],[0,520.,240],[0,0,1]])
    (args.folder/"camera.yaml").write_text(settings_text(K,np.zeros(5),640,480).replace(
        "%YAML:1.0\n", "%YAML:1.0\n# PROCEDURAL TEST CAMERA ONLY\n", 1))
    source=np.float32([[0,0],[47,0],[47,47],[0,47]])
    offsets=np.array([[-.12,-.12,0],[.12,-.12,0],[.12,.12,0],[-.12,.12,0]])
    with socket.socket() as server:
        server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        server.bind(("127.0.0.1",args.port))
        server.listen(1)
        print(f"Synthetic stream on {args.port}; settings {args.folder}/camera.yaml",flush=True)
        with server.accept()[0] as client:
            start=time.monotonic()
            for i in range(args.frames):
                phase=i/(args.frames-1)*2*np.pi
                camera=np.array([.6*np.sin(phase),.13*np.sin(phase*2),.15*(1-np.cos(phase))])
                rotation=np.array([0,.04*np.sin(phase),0.])
                R,_=cv2.Rodrigues(rotation)
                translation=-R@camera
                image=np.full((480,640,3),135,np.uint8)
                for j in np.argsort(-points[:,2]):
                    projected,_=cv2.projectPoints(points[j]+offsets,rotation,translation,K,None)
                    corners=projected.reshape(4,2).astype(np.float32)
                    if corners.max(axis=0)[0]<0 or corners.min(axis=0)[0]>640 or corners.max(axis=0)[1]<0 or corners.min(axis=0)[1]>480:
                        continue
                    H=cv2.getPerspectiveTransform(source,corners)
                    lower=np.maximum(np.floor(corners.min(axis=0)).astype(int),[0,0])
                    upper=np.minimum(np.ceil(corners.max(axis=0)).astype(int)+1,[640,480])
                    x0,y0=lower; x1,y1=upper
                    if x1<=x0 or y1<=y0:
                        continue
                    shift=np.array([[1,0,-x0],[0,1,-y0],[0,0,1]])
                    warped=cv2.warpPerspective(textures[j],shift@H,(x1-x0,y1-y0))
                    mask=cv2.warpPerspective(np.full((48,48),255,np.uint8),shift@H,(x1-x0,y1-y0))
                    region=image[y0:y1,x0:x1]
                    region[mask>200]=warped[mask>200]
                _,jpeg=cv2.imencode(".jpg",image,[cv2.IMWRITE_JPEG_QUALITY,95])
                if i==0:
                    (args.folder/"scene.jpg").write_bytes(jpeg.tobytes())
                try:
                    client.sendall(struct.pack("!4sQQIHHI",b"PSL1",i,1_000_000_000+i*50_000_000,len(jpeg),640,480,0x2000)+jpeg.tobytes())
                except (BrokenPipeError,ConnectionResetError):
                    break
                remaining=start+(i+1)*.05-time.monotonic()
                if remaining>0:
                    time.sleep(remaining)


if __name__=="__main__":
    main()
