"""Bounded CPU-only learned-depth/TSDF experiment, isolated from tracking."""
import argparse
import csv
import json
import os
from pathlib import Path
import threading
import time
import cv2
import numpy as np


def align_depth(prediction, observations):
    obs = np.asarray(observations, dtype=float)
    if obs.ndim != 2 or len(obs) < 30:
        raise ValueError("Need at least 30 SLAM depth anchors")
    h, w = prediction.shape
    uv = np.rint(obs[:, :2]).astype(int)
    good = np.isfinite(obs).all(axis=1) & (obs[:, 2] > 0) & (uv[:, 0]>=0) & (uv[:, 0]<w) & (uv[:, 1]>=0) & (uv[:, 1]<h)
    uv, obs = uv[good], obs[good]
    if len(obs)<30 or np.ptp(uv[:,0])<w*.25 or np.ptp(uv[:,1])<h*.25:
        raise ValueError("Insufficient spatial anchor coverage")
    x, y = prediction[uv[:,1],uv[:,0]], 1/obs[:,2]
    valid = np.isfinite(x) & np.isfinite(y)
    x,y=x[valid],y[valid]
    if len(x)<30 or np.std(x)<1e-6:
        raise ValueError("Degenerate inverse depth")
    # Hold out every fifth anchor; fit scale AND shift in inverse-depth space.
    train=np.arange(len(x))%5!=0
    A=np.column_stack((x,np.ones(len(x))))
    keep=train.copy()
    for _ in range(5):
        ab=np.linalg.lstsq(A[keep],y[keep],rcond=None)[0]
        relative=np.abs(A@ab-y)/y
        keep=train & (relative<.2)
        if keep.sum()<20: raise ValueError("Scale alignment has too few inliers")
    error=float(np.median(relative[~train]))
    if ab[0]<=0 or error>.15 or np.mean(relative<.2)<.65:
        raise ValueError(f"Rejected alignment, held-out relative error {error:.3f}")
    inv=prediction*ab[0]+ab[1]
    depth=np.zeros_like(inv,dtype=np.float32)
    mask=np.isfinite(inv)&(inv>0)
    depth[mask]=1/inv[mask]
    near,far=np.percentile(obs[:,2],[5,95])
    depth[(depth<near*.5)|(depth>far*2)]=0
    return depth,error


class Volume:
    def __init__(self, center, length, resolution=80):
        self.n=resolution;self.voxel=length/resolution
        self.origin=np.asarray(center)-length/2
        self.xyz=np.indices((resolution,)*3,dtype=np.float32).reshape(3,-1).T*self.voxel+self.origin
        self.sdf=np.ones(len(self.xyz),np.float32);self.weight=np.zeros(len(self.xyz),np.float32)

    def integrate(self, depth, K, T):
        p=(self.xyz-T[:3,3])@T[:3,:3]
        z=p[:,2];front=z>1e-6
        uv=np.zeros((len(z),2),int)
        uv[front]=np.rint(p[front,:2]/z[front,None]*[K[0,0],K[1,1]]+[K[0,2],K[1,2]]).astype(int)
        h,w=depth.shape
        good=front&(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
        idx=np.flatnonzero(good);d=depth[uv[idx,1],uv[idx,0]]
        trunc=3*self.voxel;delta=d-z[idx]
        # Integrate only a narrow measured surface band; unknown stays unknown.
        keep=(d>0)&(np.abs(delta)<=trunc)
        idx=idx[keep];values=np.clip(delta[keep]/trunc,-1,1)
        self.sdf[idx]=(self.sdf[idx]*self.weight[idx]+values)/(self.weight[idx]+1)
        self.weight[idx]+=1

    def mesh(self):
        from skimage.measure import marching_cubes
        shape=(self.n,)*3;mask=self.weight.reshape(shape)>=2
        if not mask.any(): return [],[]
        try:
            vertices,faces,_,_=marching_cubes(self.sdf.reshape(shape),level=0,mask=mask,spacing=(self.voxel,)*3)
        except ValueError:
            return [],[]
        # Reject triangles touching unsupported voxels.
        ijk=np.clip(np.rint(vertices/self.voxel).astype(int),0,self.n-1)
        supported=mask[ijk[:,0],ijk[:,1],ijk[:,2]]
        faces=faces[np.all(supported[faces],axis=1)]
        return (vertices+self.origin).tolist(),faces.tolist()


def run_worker(run, model, stop):
    import openvino as ov
    cv2.setNumThreads(1)
    core=ov.Core()
    net=core.compile_model(str(model),"CPU",{"INFERENCE_NUM_THREADS":2,"NUM_STREAMS":1})
    calib=json.loads((run/"intrinsics.json").read_text());K=np.array(calib["K"]);D=np.array(calib["distortion"])
    cache={};last_signature=None
    while not stop.wait(1):
        try:
            state=json.loads((run/"live_map.json").read_text())
            kfs=state.get("keyframes",[])
            signature=(state["map_id"],json.dumps(kfs,sort_keys=True))
            if signature==last_signature or not kfs: continue
            started=time.monotonic();last_signature=signature
            with (run/"frames.csv").open() as f:
                rows=[r for r in csv.DictReader(f) if r.get("image")]
            accepted=[];rejected=[]
            valid_keys={(state["map_id"],kf["id"]) for kf in kfs}
            cache={key:value for key,value in cache.items() if key in valid_keys}
            for kf in kfs:
                key=(state["map_id"],kf["id"])
                if key not in cache:
                    row=min(rows,key=lambda r:abs(float(r["time_s"])-kf["time"]))
                    if abs(float(row["time_s"])-kf["time"])>.002: continue
                    path=(run/row["image"]).resolve()
                    if not path.is_relative_to(run.resolve()): raise ValueError("Invalid frame path")
                    im=cv2.imread(str(path))
                    if im is None: continue
                    im=cv2.undistort(im,K,D)
                    rgb=cv2.cvtColor(cv2.resize(im,(256,256),interpolation=cv2.INTER_CUBIC),cv2.COLOR_BGR2RGB)/255.
                    tensor=((rgb-[.485,.456,.406])/[.229,.224,.225]).transpose(2,0,1)[None].astype(np.float32)
                    pred=np.asarray(net([tensor])[net.output(0)]).squeeze()
                    cache[key]=cv2.resize(pred,(im.shape[1],im.shape[0]),interpolation=cv2.INTER_CUBIC)
                try:
                    depth,error=align_depth(cache[key],kf["observations"])
                    accepted.append((depth,np.array(kf["T"]),error))
                except ValueError as exc: rejected.append(str(exc))
            points=[];faces=[]
            if len(accepted)>=2:
                depth,T,_=accepted[0];median=float(np.median(depth[depth>0]))
                center=T[:3,3]+T[:3,2]*median
                volume=Volume(center,median*4)
                # Rebuild from current optimized poses every publication, never mix
                # old loop-corrected geometry or independent map coordinate frames.
                for depth,T,_ in accepted: volume.integrate(depth,K,T)
                points,faces=volume.mesh()
            result={"map_id":state["map_id"],"source_updated_ns":state["updated_ns"],
                    "points":points,"faces":faces,"accepted_keyframes":len(accepted),
                    "rejected":rejected,"seconds":time.monotonic()-started,
                    "label":"Predicted depth / coarse TSDF · arbitrary scale · not measured LiDAR",
                    "alignment_errors":[x[2] for x in accepted]}
            temp=run/"dense_live.tmp.json";temp.write_text(json.dumps(result,allow_nan=False));temp.replace(run/"dense_live.json")
            print(f"Depth worker: {len(accepted)} keyframes, {len(faces)} triangles, {result['seconds']:.2f}s",flush=True)
        except (OSError,ValueError,KeyError) as exc:
            print(f"Depth worker waiting/rejected: {exc}",flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path)
    parser.add_argument("--model",type=Path,default=Path("models/cpu/midas-small.xml"))
    args=parser.parse_args()
    os.nice(5)
    stop=threading.Event()
    worker=threading.Thread(target=run_worker,args=(args.run,args.model,stop),daemon=True)
    worker.start()
    try:
        while worker.is_alive(): worker.join(.5)
    except KeyboardInterrupt:
        stop.set();worker.join(timeout=10)
