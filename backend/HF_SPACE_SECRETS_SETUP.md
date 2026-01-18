# Adding Environment Variables to Hugging Face Space

## Step-by-Step Guide

### 1. Navigate to Space Settings
1. Go to: https://huggingface.co/spaces/sehrkhan873/robotic_text_book
2. Click on **Settings** tab (at the top of the page)
3. Scroll down to the **Repository secrets** section

### 2. Add Required Secrets

Click **"New secret"** for each of the following:

#### Required Secrets:

| Secret Name | Value | Description |
|-------------|-------|-------------|
| `COHERE_API_KEY` | `vBfWiWJH5MpPkyj7LkIdnCac27WlYQUvOBiqOYGG` | Cohere API key for embeddings |
| `OPENROUTER_API_KEY` | `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` | OpenRouter API key for LLM |
| `QDRANT_URL` | `https://bfd8822d-084a-47c4-940a-e269b506633f.us-east4-0.gcp.cloud.qdrant.io` | Qdrant vector database URL |
| `QDRANT_API_KEY` | `eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.4LTAfwuNADPNs6pNNQ2H3brp9dSki2n_8MRJqPPTQVo` | Qdrant API authentication key |
| `OPENROUTER_MODEL` | `xiaomi/mimo-v2-flash` | Model to use for chat responses |

#### Optional Secrets (for Authentication features):

| Secret Name | Example Value | Description |
|-------------|---------------|-------------|
| `JWT_SECRET` | `your-super-secret-jwt-key-here-change-this` | Secret key for JWT token signing |
| `DATABASE_URL` | `sqlite:///./users.db` | Database connection string |

### 3. How to Add Each Secret

For each secret:

1. Click **"New secret"** button
2. In **Name** field: Enter the secret name (e.g., `COHERE_API_KEY`)
3. In **Value** field: Paste the corresponding value
4. Click **"Add secret"** or **"Save"**
5. Repeat for all secrets

### 4. Verify Secrets Are Added

After adding all secrets, you should see them listed in the Repository secrets section.
**Note:** The values will be hidden for security.

### 5. Restart the Space

After adding secrets:
1. Scroll to the top of the Settings page
2. Look for **"Factory reboot"** or **"Restart Space"** button
3. Click it to restart the Space with the new environment variables

### 6. Verify Deployment

Once restarted, check:
```bash
curl https://sehrkhan873-robotic-text-book.hf.space/health
```

You should see all services marked as "up":
```json
{
    "status": "healthy",
    "service": "RAG Chatbot Backend",
    "version": "1.0.0",
    "services": {
        "database": "up",
        "qdrant": "up",
        "openrouter": "up"
    }
}
```

## Important Security Notes

⚠️ **CRITICAL**: The API keys shown in this guide are from your `.env.example` file. If these are your **real production keys**:

1. **DO NOT** commit this file to GitHub or share it publicly
2. Consider rotating your API keys after deployment
3. These keys are visible in this local file - ensure it's in `.gitignore`

### Key Rotation (Recommended)

For production use, you should:

1. **Cohere**: Generate a new key at https://dashboard.cohere.com/api-keys
2. **OpenRouter**: Generate a new key at https://openrouter.ai/keys
3. **Qdrant**: Generate a new key in your Qdrant Cloud dashboard
4. **JWT Secret**: Generate a random string (at least 32 characters)

```bash
# Generate a secure JWT secret:
python3 -c "import secrets; print(secrets.token_urlsafe(32))"
```

## Troubleshooting

### Space still showing rate limit errors (429)?
- This is normal if you hit OpenRouter/Cohere rate limits
- Wait a few minutes and try again
- Consider upgrading to paid tiers for higher limits

### Health check shows services as "not configured"?
- Double-check secret names match exactly (case-sensitive)
- Ensure no extra spaces in secret values
- Try factory rebooting the Space again

### ModuleNotFoundError or import errors?
- These are code issues, not secret issues
- Check that all required files are in the Space repository
- Review the Space logs for detailed error messages

## Quick Command Reference

```bash
# Test health endpoint
curl https://sehrkhan873-robotic-text-book.hf.space/health

# Test chat endpoint (may be rate limited)
curl -X POST https://sehrkhan873-robotic-text-book.hf.space/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is a robot?", "max_results": 3}'

# View API documentation
# Visit: https://sehrkhan873-robotic-text-book.hf.space/docs
```

## Summary Checklist

- [ ] Navigate to Space Settings
- [ ] Add `COHERE_API_KEY` secret
- [ ] Add `OPENROUTER_API_KEY` secret
- [ ] Add `QDRANT_URL` secret
- [ ] Add `QDRANT_API_KEY` secret
- [ ] Add `OPENROUTER_MODEL` secret
- [ ] (Optional) Add `JWT_SECRET` secret
- [ ] (Optional) Add `DATABASE_URL` secret
- [ ] Factory reboot the Space
- [ ] Test health endpoint
- [ ] Test chat endpoint
- [ ] Review and rotate API keys for production
