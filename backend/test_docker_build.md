# Docker Build Test Instructions

## The Fix Applied
Fixed the permission error by:
1. Creating `appuser` before copying files
2. Copying Python packages to `/home/appuser/.local` instead of `/root/.local`
3. Updating PATH to `/home/appuser/.local/bin`
4. Setting proper ownership with `chown -R appuser:appuser /home/appuser/.local`

## Local Testing (if Docker Desktop is running)

### Build the image:
```bash
docker build -t seher-ai-backend:test .
```

### Test run locally:
```bash
docker run -p 7860:7860 \
  -e COHERE_API_KEY="your_key" \
  -e OPENROUTER_API_KEY="your_key" \
  -e QDRANT_URL="your_url" \
  -e QDRANT_API_KEY="your_key" \
  -e OPENROUTER_MODEL="google/gemini-flash-1.5" \
  seher-ai-backend:test
```

### Verify it's working:
```bash
curl http://localhost:7860/health
```

## Deploy to Hugging Face Spaces

### Option 1: Via Web Interface
1. Go to your Space on Hugging Face
2. Upload the fixed `Dockerfile`
3. The Space will automatically rebuild

### Option 2: Via Git
```bash
# Clone your space (if not already)
git clone https://huggingface.co/spaces/YOUR_USERNAME/YOUR_SPACE_NAME
cd YOUR_SPACE_NAME

# Copy the fixed Dockerfile
cp /path/to/backend/Dockerfile .

# Commit and push
git add Dockerfile
git commit -m "fix: resolve permission denied error for uvicorn

- Copy Python packages to /home/appuser/.local instead of /root/.local
- Update PATH to point to appuser's local bin
- Set proper ownership before switching to non-root user"

git push
```

## Expected Build Output
The build should complete without errors and you should see:
- ✓ Stage 1 completes successfully
- ✓ Stage 2 creates appuser
- ✓ Packages copied to /home/appuser/.local
- ✓ Ownership set correctly
- ✓ Container starts with uvicorn command

## Health Check
Once deployed, verify:
```bash
curl https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/health
```

Expected response:
```json
{
  "status": "healthy",
  "services": {
    "cohere": "configured/not configured",
    "qdrant": "configured/not configured"
  }
}
```

## Troubleshooting

If you still see permission errors:
1. Check the build logs for the COPY and chown commands
2. Verify PATH is set to `/home/appuser/.local/bin`
3. Ensure packages were installed with `--user` flag in builder stage
