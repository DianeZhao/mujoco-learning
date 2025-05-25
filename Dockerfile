# Use Ubuntu 22.04 base image
FROM ubuntu:22.04

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3.10 python3.10-venv python3-pip \
    python3.10-tk \
    git curl build-essential \
    libgl1-mesa-glx libgl1-mesa-dri libglfw3 libx11-6 \
    && rm -rf /var/lib/apt/lists/*

# Use python3.10 as default python
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1

# Upgrade pip and install wheel
RUN python3 -m pip install --upgrade pip setuptools wheel

# Copy your requirements.txt
COPY requirements.txt /tmp/

# Install requirements
RUN python3 -m pip install -r /tmp/requirements.txt

# Create a working directory
WORKDIR /workspace

# Copy your project files (optional, depends on workflow)
COPY . /workspace

# Default command (optional, open bash)
CMD ["/bin/bash"]
