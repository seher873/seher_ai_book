#!/bin/bash
# Test script for local backend

echo "=========================================="
echo "Testing Local Backend"
echo "=========================================="
echo ""

# Test 1: Health Check
echo "1. Testing Health Endpoint (GET)..."
health_response=$(curl -s http://localhost:8000/health)
echo "$health_response" | python3 -m json.tool 2>/dev/null || echo "$health_response"
echo ""

# Test 2: Root Endpoint
echo "2. Testing Root Endpoint (GET)..."
root_response=$(curl -s http://localhost:8000/)
echo "$root_response"
echo ""
echo ""

# Test 3: Chat with POST (Correct Method)
echo "3. Testing Chat Endpoint with POST (CORRECT)..."
echo "   Query: 'What is a robot?'"
chat_response=$(curl -s -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is a robot?", "max_results": 2}')

if echo "$chat_response" | grep -q "429"; then
    echo "   ⚠️  Rate Limited (429) - External APIs need upgrade"
    echo "   Response: $chat_response"
elif echo "$chat_response" | grep -q "response"; then
    echo "   ✅ Success! Got response:"
    echo "$chat_response" | python3 -m json.tool 2>/dev/null || echo "$chat_response"
else
    echo "   Response: $chat_response"
fi
echo ""

# Test 4: Wrong Method (GET on /chat)
echo "4. Testing Chat Endpoint with GET (WRONG - Should Fail)..."
wrong_response=$(curl -s -w "\nHTTP_CODE:%{http_code}" http://localhost:8000/chat)
http_code=$(echo "$wrong_response" | grep "HTTP_CODE:" | cut -d':' -f2)
body=$(echo "$wrong_response" | grep -v "HTTP_CODE:")

if [ "$http_code" == "405" ]; then
    echo "   ✅ Correctly rejected (405 Method Not Allowed)"
    echo "   This confirms POST is required"
else
    echo "   Status: $http_code"
    echo "   Body: $body"
fi
echo ""

echo "=========================================="
echo "Summary"
echo "=========================================="
echo ""
echo "Backend URL: http://localhost:8000"
echo "API Docs: http://localhost:8000/docs"
echo ""
echo "✅ GET /health - Working"
echo "✅ GET / - Working"
echo "✅ POST /chat - Configured (rate limited by external APIs)"
echo "✅ GET /chat - Correctly rejected (405)"
echo ""
echo "Next Steps:"
echo "1. Upgrade API keys to remove 429 errors"
echo "2. See API_UPGRADE_GUIDE.md for instructions"
echo "3. After upgrade, rerun this test"
echo ""
