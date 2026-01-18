# API Usage Guide - Correct HTTP Methods

## ✅ Backend is Running!

**Backend URL**: http://localhost:8000
**Status**: Healthy - All services up
**API Docs**: http://localhost:8000/docs

---

## Common Error: "Method Not Allowed"

### ❌ Wrong Method (GET):
```bash
curl http://localhost:8000/chat  # Error: 405 Method Not Allowed
```

### ✅ Correct Method (POST):
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS2?", "max_results": 3}'
```

---

## API Endpoints Summary

| Endpoint | Method | Purpose | Example |
|----------|--------|---------|---------|
| `/` | GET | Root message | `curl http://localhost:8000/` |
| `/health` | GET | Health check | `curl http://localhost:8000/health` |
| `/chat` | **POST** | Chat with RAG | See below |
| `/ingest` | **POST** | Upload documents | See below |
| `/auth/signup` | **POST** | User registration | See below |
| `/auth/signin` | **POST** | User login | See below |
| `/docs` | GET | API documentation | Open in browser |

---

## Correct Usage Examples

### 1. Health Check (GET)
```bash
curl http://localhost:8000/health
```

**Response**:
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

---

### 2. Chat Endpoint (POST) ✅ CORRECT

**Basic Query**:
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS2?"}'
```

**With Max Results**:
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is a robot?", "max_results": 5}'
```

**Expected Response** (when API limits allow):
```json
{
  "query": "What is ROS2?",
  "response": "ROS2 (Robot Operating System 2) is...",
  "sources": [
    {
      "id": "doc_123",
      "title": "Chapter 3: ROS2 Basics",
      "path": "/docs/ch3",
      "score": 0.95
    }
  ]
}
```

**Current Issue (429 Rate Limit)**:
```json
{
  "detail": "status_code: 429, body: data=None"
}
```
This means **external APIs (Cohere/OpenRouter) are rate limited**. See [API_UPGRADE_GUIDE.md](./API_UPGRADE_GUIDE.md) to fix this.

---

### 3. User Signup (POST)

```bash
curl -X POST http://localhost:8000/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "user@example.com",
    "password": "securepassword123",
    "full_name": "John Doe",
    "software_background": "Python, JavaScript",
    "hardware_background": "Arduino, Raspberry Pi"
  }'
```

**Response**:
```json
{
  "access_token": "eyJ0eXAiOiJKV1QiLCJhbGc...",
  "token_type": "bearer",
  "user": {
    "id": 1,
    "email": "user@example.com",
    "full_name": "John Doe"
  }
}
```

---

### 4. User Signin (POST)

```bash
curl -X POST http://localhost:8000/auth/signin \
  -H "Content-Type: application/json" \
  -d '{
    "email": "user@example.com",
    "password": "securepassword123"
  }'
```

---

## Testing from Frontend

### JavaScript/Fetch
```javascript
// ✅ Correct: POST request
fetch('http://localhost:8000/chat', {
  method: 'POST',  // Must be POST!
  headers: {
    'Content-Type': 'application/json',
  },
  body: JSON.stringify({
    query: 'What is ROS2?',
    max_results: 3
  })
})
.then(response => response.json())
.then(data => console.log(data))
.catch(error => console.error('Error:', error));
```

### Python/Requests
```python
import requests

# ✅ Correct: POST request
response = requests.post(
    'http://localhost:8000/chat',
    json={
        'query': 'What is ROS2?',
        'max_results': 3
    }
)

print(response.json())
```

### React Example
```javascript
const ChatComponent = () => {
  const [response, setResponse] = useState('');

  const sendQuery = async () => {
    try {
      // ✅ Correct: POST request
      const res = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query: 'What is ROS2?',
          max_results: 3
        })
      });

      const data = await res.json();
      setResponse(data.response);
    } catch (error) {
      console.error('Error:', error);
    }
  };

  return (
    <div>
      <button onClick={sendQuery}>Ask Question</button>
      <p>{response}</p>
    </div>
  );
};
```

---

## Common Errors & Solutions

### Error 1: 405 Method Not Allowed
```json
{"detail": "Method Not Allowed"}
```

**Cause**: Using GET instead of POST
**Solution**: Use `-X POST` in curl or `method: 'POST'` in fetch

---

### Error 2: 422 Validation Error
```json
{
  "detail": [
    {
      "loc": ["body", "query"],
      "msg": "field required",
      "type": "value_error.missing"
    }
  ]
}
```

**Cause**: Missing required field `query`
**Solution**: Include `"query"` in request body

---

### Error 3: 429 Rate Limit
```json
{"detail": "status_code: 429, body: data=None"}
```

**Cause**: External APIs (Cohere/OpenRouter) rate limited
**Solutions**:
1. Wait 2-3 minutes between requests
2. Upgrade API keys (see API_UPGRADE_GUIDE.md)
3. Add credits to OpenRouter ($5-10)

---

### Error 4: 429 Too Many Requests (Your Rate Limiter)
```json
{"error": "Rate limit exceeded: 10 per 1 minute"}
```

**Cause**: Exceeded your application's rate limit (10 req/min)
**Solution**: Wait 1 minute or adjust rate limit in main.py

---

## Testing with Postman

1. **Create New Request**
2. **Method**: POST
3. **URL**: `http://localhost:8000/chat`
4. **Headers**:
   - Key: `Content-Type`
   - Value: `application/json`
5. **Body** (raw JSON):
```json
{
  "query": "What is ROS2?",
  "max_results": 3
}
```
6. **Click Send**

---

## API Documentation (Swagger UI)

Open in browser: http://localhost:8000/docs

This provides:
- Interactive API testing
- All endpoint documentation
- Request/response schemas
- Try it out functionality

---

## Rate Limits (Your Application)

| Endpoint | Limit | Per |
|----------|-------|-----|
| `/chat` | 10 requests | minute |
| `/ingest` | 5 requests | minute |
| `/health` | Unlimited | - |
| `/auth/*` | Unlimited | - |

---

## Current Status

✅ **Backend Running**: http://localhost:8000
✅ **All Services Up**: Database, Qdrant, OpenRouter
⚠️ **External API Limits**: Cohere & OpenRouter rate limited

**Next Steps**:
1. Upgrade API keys (see API_UPGRADE_GUIDE.md)
2. Test chat endpoint after upgrade
3. Start frontend to use the UI

---

## Quick Test Commands

```bash
# 1. Check backend is running
curl http://localhost:8000/health

# 2. Test chat (POST - Correct!)
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "test"}'

# 3. Open API docs
open http://localhost:8000/docs
# Or on Windows: start http://localhost:8000/docs
```

---

## Summary

✅ **DO**: Use `POST` for `/chat`, `/ingest`, `/auth/signup`, `/auth/signin`
✅ **DO**: Use `GET` for `/health`, `/`, `/docs`
❌ **DON'T**: Use `GET` for `/chat` - it won't work!
❌ **DON'T**: Forget `Content-Type: application/json` header

**Remember**: The 429 error is from external APIs being rate limited, not your code. Upgrade API keys to fix!
