"""Mechanical API updates for pinned DPVO sources and PyTorch 2.7/CUDA 12.8."""
import re
import sys
from pathlib import Path

root = Path(sys.argv[1]).resolve()
if not (root / "dpvo/dpvo.py").is_file():
    raise SystemExit("Expected a DPVO source checkout")
for folder in ("dpvo/altcorr", "dpvo/fastba", "dpvo/lietorch/src"):
    for path in (root / folder).rglob("*"):
        if path.suffix not in (".cu", ".cpp", ".h"):
            continue
        source = path.read_text()
        source = re.sub(r"\b([A-Za-z_]\w*)\.type\(\)", r"\1.scalar_type()", source)
        source = source.replace("#include <THC/THCAtomics.cuh>", "#include <ATen/cuda/Atomic.cuh>")
        if path.name == "correlation_kernel.cu":
            source = re.sub(r"\batomicAdd\(", "gpuAtomicAdd(", source)
        path.write_text(source)
path = root / "dpvo/lietorch/include/dispatch.h"
source = path.read_text().replace("::detail::scalar_type(the_type)", "the_type")
path.write_text(source)
path = root / "dpvo/dpvo.py"
source = path.read_text().replace("torch.load(network)",
    'torch.load(network, map_location="cpu", weights_only=True)')
path.write_text(source)
print("Updated tensor dispatch, CUDA atomics, and checkpoint loading")
