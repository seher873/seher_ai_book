# Deploying to Hugging Face Spaces

This backend is optimized for Hugging Face Spaces using a Docker container.

## Deployment Steps

1. **Create a New Space**:
   - Go to [huggingface.co/new-space](https://huggingface.co/new-space).
   - Name your space (e.g., `physical-ai-backend`).
   - Select **Docker** as the SDK.
   - Choose a hardware tier (the free "CPU basic" is sufficient for this FastAPI app).

2. **Configure Environment Variables (Secrets)**:
   - Go to your Space's **Settings** tab.
   - Scroll down to **Variables and secrets**.
   - Add the following as **Secrets** (NOT Variables):
     - `COHERE_API_KEY`: Your key from Cohere.
     - `OPENROUTER_API_KEY`: Your key from OpenRouter.
     - `QDRANT_URL`: Your Qdrant Cloud URL.
     - `QDRANT_API_KEY`: Your Qdrant API key.
     - `OPENROUTER_MODEL`: Set to `google/gemini-flash-1.5` or `xiaomi/mimo-v2-flash`.

3. **Upload Files**:
   - You can use the Hugging Face Web interface or `git` to upload the contents of the `backend` directory.
   - Ensure the `Dockerfile` and `main.py` are at the root of the repository.

4. **Port Configuration**:
   - The `Dockerfile` is already configured to use port `7860`, which Hugging Face expects.

## Health Check
Once deployed, your backend will be available at:
`https://<your-username>-<your-space-name>.hf.space`

You can verify it's working by visiting:
`https://<your-username>-<your-space-name>.hf.space/health`
