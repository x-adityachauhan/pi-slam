#!/usr/bin/env python3
import json
import torch

if not torch.cuda.is_available():
    raise SystemExit("CUDA is unavailable. Check the NVIDIA driver and NVIDIA Container Toolkit.")
capability = torch.cuda.get_device_capability()
free, total = torch.cuda.mem_get_info()
print(json.dumps({"torch": torch.__version__, "cuda": torch.version.cuda,
    "gpu": torch.cuda.get_device_name(), "compute_capability": capability,
    "free_vram_gib": free / 2**30, "total_vram_gib": total / 2**30}, indent=2))
if capability != (12, 0):
    raise SystemExit("This package targets RTX 5060 / sm_120. Rebuild for this GPU's architecture.")
if free < 4 * 2**30:
    raise SystemExit("Less than 4 GiB of GPU memory is free. Close other GPU applications first.")
import cuda_ba
import cuda_corr
import lietorch_backends
from dpvo.lietorch import SE3

identity = SE3.Identity(2, device="cuda")
assert torch.isfinite(identity.inv().data).all()
from dpvo.dpvo import DPVO
from dpvo.config import cfg

config = cfg.clone()
config.BUFFER_SIZE = 2048
config.PATCHES_PER_FRAME = 96
config.LOOP_CLOSURE = True
config.MAX_EDGE_AGE = 512
model = DPVO(config, "/opt/weights/dpvo.pth", ht=480, wd=640, viz=False)
image = torch.zeros(3, 480, 640, dtype=torch.uint8, device="cuda")
intrinsics = torch.tensor([520., 520., 320., 240.], device="cuda")
model(0.0, image, intrinsics)
model(0.05, image, intrinsics)
torch.cuda.synchronize()
print("Checkpoint, CUDA Lie algebra, patch extraction, and correlation extensions loaded successfully.")
