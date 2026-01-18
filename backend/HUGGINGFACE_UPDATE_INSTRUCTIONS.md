# How to Update Dockerfile on Hugging Face Space

## The Issue
The current Dockerfile on your Hugging Face Space has a permission error where `appuser` cannot access `/root/.local/bin/uvicorn`.

## Quick Fix via Web Interface (Recommended)

1. Go to https://huggingface.co/spaces/sehrkhan873/robotic_text_book
2. Click on **Files** tab
3. Click on **Dockerfile**
4. Click the **Edit** button (pencil icon)
5. Replace lines 21-43 with the following:

```dockerfile
# Stage 2: Runtime - Minimal production image
FROM python:3.10-slim

# Set working directory
WORKDIR /app

# Create non-root user for security first
RUN useradd -m -u 1000 appuser

# Copy Python packages from builder to appuser's home
COPY --from=builder /root/.local /home/appuser/.local

# Copy application code
COPY . .

# Change ownership of all files to appuser
RUN chown -R appuser:appuser /app /home/appuser/.local

# Set environment variables
ENV PYTHONPATH=/app \
    PORT=7860 \
    PATH=/home/appuser/.local/bin:$PATH \
    PYTHONUNBUFFERED=1
```

6. Scroll down and commit the changes

## What Changed?

### Before (Lines 21-40):
```dockerfile
# Stage 2: Runtime - Minimal production image
FROM python:3.10-slim

# Set working directory
WORKDIR /app

# Copy Python packages from builder
COPY --from=builder /root/.local /root/.local  # ← Wrong location

# Copy application code
COPY . .

# Create non-root user for security
RUN useradd -m -u 1000 appuser && \
    chown -R appuser:appuser /app  # ← Missing /home/appuser/.local

# Set environment variables
ENV PYTHONPATH=/app \
    PORT=7860 \
    PATH=/root/.local/bin:$PATH \  # ← Wrong PATH
    PYTHONUNBUFFERED=1
```

### After (Fixed):
```dockerfile
# Stage 2: Runtime - Minimal production image
FROM python:3.10-slim

# Set working directory
WORKDIR /app

# Create non-root user for security first  # ← Created first
RUN useradd -m -u 1000 appuser

# Copy Python packages from builder to appuser's home
COPY --from=builder /root/.local /home/appuser/.local  # ← Correct location

# Copy application code
COPY . .

# Change ownership of all files to appuser
RUN chown -R appuser:appuser /app /home/appuser/.local  # ← Includes .local

# Set environment variables
ENV PYTHONPATH=/app \
    PORT=7860 \
    PATH=/home/appuser/.local/bin:$PATH \  # ← Correct PATH
    PYTHONUNBUFFERED=1
```

## Alternative: Clone Space and Update

If you prefer using git:

```bash
# Clone your Space to a new directory
git clone https://huggingface.co/spaces/sehrkhan873/robotic_text_book hf_space
cd hf_space

# Edit the Dockerfile (use the changes above)
nano Dockerfile  # or your preferred editor

# Commit and push
git add Dockerfile
git commit -m "fix: resolve uvicorn permission denied error"
git push
```

## Verification

After updating, the Space will automatically rebuild. Check the build logs to ensure:
1. No permission errors
2. Container starts successfully
3. Health check at `/health` returns 200 OK

Visit: https://sehrkhan873-robotic-text-book.hf.space/health
