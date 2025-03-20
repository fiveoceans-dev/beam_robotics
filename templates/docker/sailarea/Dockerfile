# Use PyTorch image for ARM64 (Apple Silicon)
FROM --platform=linux/arm64 pytorch/pytorch:2.0.1-cuda11.7-cudnn8-runtime

WORKDIR /app

# Prevents Python from writing .pyc files
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

# Install system dependencies required for OpenCV
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 && \
    rm -rf /var/lib/apt/lists/*

# Set PyTorch cache directory
ENV TORCH_HOME=/app/.cache/torch

# Copy dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy source code
COPY . .

# Set up a non-root user
RUN groupadd --system appuser && useradd --system --gid appuser appuser
RUN chown -R appuser:appuser /app
USER appuser

# Expose WebSocket ports
EXPOSE 3007 3008

# Start WebSocket server
CMD ["python", "inference.py"]
