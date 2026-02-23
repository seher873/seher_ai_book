# Deploy Updated Backend to Hugging Face Space

## Problem
The Hugging Face Space is running outdated code with an old Qdrant client API (`search_points` method that no longer exists).

## Solution: Push Updated Code via Git

### Step 1: Clone your Hugging Face Space

```bash
# Create a temporary directory for the HF Space
cd /tmp
git clone https://huggingface.co/spaces/sehrkhan873/robotic_text_book hf_space_deploy
cd hf_space_deploy
```

**Note:** You'll need your Hugging Face token for authentication. Use:
```bash
git config credential.helper store
```

### Step 2: Copy Updated Files

Copy the updated backend files from your project:

```bash
# From your project directory
cp /mnt/c/Users/user/Desktop/text-book/seher_ai_book/backend/main.py /tmp/hf_space_deploy/
cp /mnt/c/Users/user/Desktop/text-book/seher_ai_book/backend/Dockerfile /tmp/hf_space_deploy/
cp /mnt/c/Users/user/Desktop/text-book/seher_ai_book/backend/requirements.txt /tmp/hf_space_deploy/
cp -r /mnt/c/Users/user/Desktop/text-book/seher_ai_book/backend/auth /tmp/hf_space_deploy/
```

### Step 3: Commit and Push

```bash
cd /tmp/hf_space_deploy

# Add all files
git add .

# Commit with descriptive message
git commit -m "fix: update Qdrant client API (search_points -> search)

- Replace deprecated search_points method with search method
- Use query_vector parameter for qdrant-client>=1.12.0
- Fix AttributeError in chat endpoint"

# Push to Hugging Face Space
git push
```

### Step 4: Monitor Deployment

1. Go to: https://huggingface.co/spaces/sehrkhan873/robotic_text_book
2. Click on **"App file"** or **"Logs"** tab to see build progress
3. Wait for the build to complete (usually 2-5 minutes)
4. The Space will automatically restart with the new code

### Step 5: Verify the Fix

Test the chat endpoint:

```bash
curl -X POST https://sehrkhan873-robotic-text-book.hf.space/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is Physical AI?", "max_results": 3}'
```

Expected response:
```json
{
  "query": "What is Physical AI?",
  "response": "...",
  "sources": [...]
}
```

---

## Option 2: Via Hugging Face Web Interface

If you prefer not to use git:

1. Go to: https://huggingface.co/spaces/sehrkhan873/robotic_text_book
2. Click on **"Files"** tab
3. Click on **main.py**
4. Click **Edit** (pencil icon)
5. Replace the entire content with the updated main.py
6. Scroll down and commit with message: `fix: update Qdrant client API`
7. Repeat for **Dockerfile** and **requirements.txt** if needed
8. The Space will automatically rebuild

---

## What Was Fixed

### Before (Broken):
```python
# Tried deprecated search_points method
search_results = qdrant.search_points(
    collection_name=COLLECTION_NAME,
    vector=query_embedding,
    limit=chat_request.max_results,
    with_payload=True
)
```

### After (Fixed):
```python
# Use correct search method for qdrant-client>=1.12.0
search_results = qdrant.search(
    collection_name=COLLECTION_NAME,
    query_vector=query_embedding,
    limit=chat_request.max_results,
    with_payload=True
)
```

---

## Troubleshooting

### Build Fails
Check the build logs in the "Logs" tab of your Space. Common issues:
- Missing environment variables (check Secrets)
- Docker build errors (check Dockerfile syntax)
- Python package conflicts (check requirements.txt)

### Chat Still Returns Error
1. Wait a few minutes for the Space to fully restart
2. Check the logs for any new errors
3. Verify all environment variables are set correctly

### Environment Variables
Make sure these Secrets are configured in your Space:
- `COHERE_API_KEY`
- `OPENROUTER_API_KEY`
- `OPENROUTER_MODEL` (e.g., `google/gemini-flash-1.5`)
- `QDRANT_URL`
- `QDRANT_API_KEY`
- `JWT_SECRET`
- `DATABASE_URL`

Set them at: https://huggingface.co/spaces/sehrkhan873/robotic_text_book/settings
