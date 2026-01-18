# API Upgrade Guide - Remove Rate Limits

This guide explains how to upgrade your API keys to paid tiers to remove or significantly increase rate limits for your RAG chatbot.

---

## Current Situation

You're experiencing **429 (Too Many Requests)** errors from:
- **Cohere API** (for embeddings)
- **OpenRouter API** (for LLM responses)

These are **external API rate limits**, not from your application. Your application's rate limiting (10 req/min) is working correctly, but the external APIs have their own stricter limits on free tiers.

---

## Services to Upgrade

### 1. Cohere API (Embeddings)

#### Current Status
- **Free Tier**: Very limited requests (trial tier)
- **Model Used**: `embed-english-v3.0`
- **Usage**: Every chat query requires 1 embedding call

#### Upgrade Options

**Option A: Trial Tier (Free for 2 months)**
- **Cost**: FREE for 2 months
- **Rate Limit**: 100 requests/minute
- **Monthly Quota**: No hard limit during trial
- **How to Upgrade**:
  1. Go to https://dashboard.cohere.com/
  2. Navigate to **Settings** → **Billing**
  3. Add payment method (credit card)
  4. No charges for 2 months
  5. Automatically activates trial tier

**Option B: Production Tier**
- **Cost**: Pay-as-you-go after trial
- **Pricing**: ~$0.0001 per 1K tokens (very affordable)
- **Rate Limit**: 10,000 requests/minute
- **How to Upgrade**: Same as Trial, but remains active after trial period

#### Steps to Upgrade Cohere:

```bash
1. Visit: https://dashboard.cohere.com/
2. Sign in with your account
3. Go to: Settings → Billing
4. Click "Add Payment Method"
5. Enter credit card details
6. Confirm trial tier activation
7. Your existing API key will automatically get upgraded limits
```

**No need to change your API key** - the same key gets upgraded limits!

---

### 2. OpenRouter API (LLM Responses)

#### Current Status
- **Free Tier**: Extremely limited (50-100 requests/day for free models)
- **Model Used**: `xiaomi/mimo-v2-flash` (free model)
- **Usage**: Every chat query requires 1 LLM call

#### Upgrade Options

**Option A: Add Credits (Recommended)**
- **Minimum**: $5 USD
- **Typical Usage Cost**: ~$0.001 - $0.01 per query (depending on model)
- **Estimated Queries**: 500-5000 queries per $5 (varies by model)
- **Rate Limits**: Much higher (100-1000 req/min depending on credits)

**Option B: Use Free Models with Better Limits**
Some free models have higher rate limits:
- `google/gemini-flash-1.5-8b` - Better free tier limits
- `meta-llama/llama-3.2-1b-instruct:free` - Higher free quota

**How to Add Credits:**

```bash
1. Visit: https://openrouter.ai/credits
2. Sign in to your account
3. Click "Add Credits"
4. Choose amount: $5, $10, $20, or custom
5. Complete payment (credit card, crypto, etc.)
6. Credits appear immediately
7. Your existing API key will use these credits
```

**Cost Breakdown for Popular Models:**

| Model | Cost per 1M input tokens | Cost per chat (~500 tokens) |
|-------|-------------------------|----------------------------|
| `xiaomi/mimo-v2-flash` | FREE (rate limited) | $0 |
| `google/gemini-flash-1.5` | $0.075 | ~$0.0004 |
| `anthropic/claude-3-haiku` | $0.25 | ~$0.0015 |
| `openai/gpt-3.5-turbo` | $0.50 | ~$0.003 |

**$5 credit gets you:**
- ~12,500 queries with Gemini Flash
- ~3,300 queries with Claude Haiku
- ~1,650 queries with GPT-3.5

---

### 3. Qdrant (Vector Database)

#### Current Status
- **Free Tier**: 1GB storage, good limits
- **Current Usage**: Should be sufficient
- **Rate Limits**: 100 requests/second (generous)

#### When to Upgrade
- Only if you hit storage limits (1GB)
- Or need more than 100 req/sec
- Typically NOT the bottleneck

**Upgrade if needed:**
- https://cloud.qdrant.io/
- Upgrade to paid cluster ($25+/month)

---

## Recommended Upgrade Path

### Step 1: Add OpenRouter Credits (PRIORITY)
**Why First**: This is likely the main bottleneck
**Cost**: $5-10 USD
**Impact**: Removes most 429 errors

```bash
1. Go to: https://openrouter.ai/credits
2. Add $10 credits (recommended for testing)
3. Keep using same API key
```

### Step 2: Upgrade Cohere to Trial
**Why Second**: Free for 2 months
**Cost**: FREE (requires credit card)
**Impact**: 100x higher rate limits

```bash
1. Go to: https://dashboard.cohere.com/billing
2. Add payment method
3. Activate trial tier
4. Keep using same API key
```

### Step 3: Update Environment Variables (If needed)
If you generate new API keys, update them:

**Local (.env file):**
```bash
COHERE_API_KEY="your_new_key_here"
OPENROUTER_API_KEY="your_new_key_here"
```

**Hugging Face Space:**
```bash
1. Go to: https://huggingface.co/spaces/sehrkhan873/robotic_text_book/settings
2. Update Repository secrets
3. Factory reboot the Space
```

---

## Cost Estimates

### Scenario: Moderate Usage
- **100 queries/day**
- **30 days/month**
- **Total**: 3,000 queries/month

**Monthly Costs:**
- **Cohere**: FREE (trial) or ~$0.30/month (production)
- **OpenRouter** (Gemini Flash): ~$1.20/month
- **Qdrant**: FREE (within 1GB)
- **Total**: ~$1.50/month

### Scenario: Heavy Usage
- **500 queries/day**
- **30 days/month**
- **Total**: 15,000 queries/month

**Monthly Costs:**
- **Cohere**: ~$1.50/month
- **OpenRouter** (Gemini Flash): ~$6.00/month
- **Qdrant**: FREE (if within 1GB)
- **Total**: ~$7.50/month

### Scenario: Production
- **2,000 queries/day**
- **30 days/month**
- **Total**: 60,000 queries/month

**Monthly Costs:**
- **Cohere**: ~$6/month
- **OpenRouter** (Gemini Flash): ~$24/month
- **Qdrant**: May need paid tier ($25/month)
- **Total**: ~$30-55/month

---

## Alternative: Use Different Models

### Free Models with Better Limits

Instead of upgrading, you could switch to models with better free tiers:

#### Option 1: Google Gemini (Free)
```python
# In main.py, change:
GENERATION_MODEL = "google/gemini-flash-1.5-8b"  # Better free limits
```

#### Option 2: Meta Llama (Free)
```python
GENERATION_MODEL = "meta-llama/llama-3.2-3b-instruct:free"
```

#### Option 3: Mistral (Free)
```python
GENERATION_MODEL = "mistralai/mistral-7b-instruct:free"
```

**How to Change:**
1. Update `OPENROUTER_MODEL` in your `.env` file
2. Update the secret in Hugging Face Space settings
3. Restart the Space

---

## Testing After Upgrade

### Test 1: Single Query
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS2?", "max_results": 3}'
```

### Test 2: Multiple Queries
```bash
# Should not hit 429 errors anymore
for i in {1..10}; do
  curl -X POST http://localhost:8000/chat \
    -H "Content-Type: application/json" \
    -d "{\"query\": \"Test query $i\"}"
  echo ""
done
```

### Test 3: Check Usage
- **Cohere**: https://dashboard.cohere.com/usage
- **OpenRouter**: https://openrouter.ai/activity
- **Qdrant**: https://cloud.qdrant.io/

---

## Monitoring Costs

### Cohere Dashboard
```bash
URL: https://dashboard.cohere.com/usage
Shows:
- Requests per day
- Tokens consumed
- Cost (if on paid tier)
```

### OpenRouter Activity
```bash
URL: https://openrouter.ai/activity
Shows:
- Remaining credits
- Cost per request
- Model usage breakdown
```

### Set Up Alerts
Both services allow you to set spending alerts:
1. **Cohere**: Settings → Billing → Set budget alerts
2. **OpenRouter**: Credits page → Set low balance alerts

---

## Quick Start Checklist

- [ ] **Step 1**: Go to https://openrouter.ai/credits
- [ ] **Step 2**: Add $10 credits (will last 2-4 weeks of testing)
- [ ] **Step 3**: Go to https://dashboard.cohere.com/billing
- [ ] **Step 4**: Add payment method (activates free trial)
- [ ] **Step 5**: Test your chatbot (should work without 429 errors)
- [ ] **Step 6**: Monitor usage in first week
- [ ] **Step 7**: Adjust credits/tier based on actual usage

---

## Current API Keys Status

Based on your `.env` file:

```
COHERE_API_KEY: vBfWiWJH5MpPkyj7LkIdnCac27WlYQUvOBiqOYGG
OPENROUTER_API_KEY: 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
```

**Action Required:**
1. Log in to both services with these keys
2. Check current tier/credits
3. Upgrade as needed
4. Keys will automatically get upgraded limits (no need to regenerate)

---

## Support Resources

### Cohere Support
- **Docs**: https://docs.cohere.com/
- **Pricing**: https://cohere.com/pricing
- **Discord**: https://discord.gg/cohere

### OpenRouter Support
- **Docs**: https://openrouter.ai/docs
- **Pricing**: https://openrouter.ai/models (per model)
- **Discord**: https://discord.gg/openrouter

---

## Summary

**Fastest Solution**: Add $10 to OpenRouter + Activate Cohere Trial

**Total Cost**: $10 (one-time) + $0 (trial)

**Result**: Removes all 429 rate limit errors

**Time**: 5-10 minutes to set up

**Estimated Duration**:
- OpenRouter credits: 2-4 weeks of moderate testing
- Cohere trial: 2 months free

---

**Need Help?** Let me know which service you want to upgrade first, and I can provide more detailed step-by-step instructions!
