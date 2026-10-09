FROM pytorch/pytorch:2.7.1-cuda12.8-cudnn9-devel
ENV DEBIAN_FRONTEND=noninteractive \
    TORCH_CUDA_ARCH_LIST="12.0+PTX" \
    FORCE_CUDA=1 \
    MAX_JOBS=2 \
    OMP_NUM_THREADS=4 \
    OPENBLAS_NUM_THREADS=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/opt/DPVO:/opt/pi-slam
RUN apt-get update && apt-get install -y --no-install-recommends \
    git curl unzip build-essential libeigen3-dev libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/pi-slam
COPY gpu/requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt
RUN git clone https://github.com/princeton-vl/DPVO.git /opt/DPVO \
    && git -C /opt/DPVO checkout 0ac95b656d1fda91c271d2a106460d19ad966fc7 \
    && mkdir -p /opt/DPVO/thirdparty \
    && ln -s /usr/include/eigen3 /opt/DPVO/thirdparty/eigen-3.4.0
COPY gpu/modernize_sources.py /tmp/modernize_sources.py
RUN python /tmp/modernize_sources.py /opt/DPVO \
    && pip install --no-cache-dir --no-build-isolation --no-deps /opt/DPVO \
    && pip install --no-cache-dir --no-build-isolation --no-deps torch-scatter==2.1.2
# Upstream pretrained weights, bundled with their checksum and provenance.
COPY gpu/weights/dpvo.pth /opt/weights/dpvo.pth
RUN echo '30d02dc2b88a321cf99aad8e4ea1152a44d791b5b65bf95ad036922819c0ff12  /opt/weights/dpvo.pth' | sha256sum -c -
COPY stream.py /opt/pi-slam/stream.py
COPY gpu /opt/pi-slam/gpu
COPY viewer /opt/pi-slam/viewer
COPY rc /opt/pi-slam/rc
WORKDIR /opt/pi-slam
CMD ["python", "gpu/live_dpvo.py", "--calibration", "/data/calibration/intrinsics.json", "--output", "/data/runs"]
