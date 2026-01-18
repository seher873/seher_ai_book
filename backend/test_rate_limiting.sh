#!/bin/bash
# Test script to verify rate limiting on Hugging Face Space

SPACE_URL="https://sehrkhan873-robotic-text-book.hf.space"

echo "=========================================="
echo "Testing Rate Limiting Implementation"
echo "=========================================="
echo ""

echo "Rate Limits Configured:"
echo "  /chat:   10 requests per minute"
echo "  /ingest: 5 requests per minute"
echo ""
echo "=========================================="
echo ""

# Test 1: Send multiple requests to test rate limit
echo "Test 1: Sending 3 chat requests rapidly..."
echo ""

for i in {1..3}; do
    echo "Request $i:"
    response=$(curl -s -w "\nHTTP_CODE:%{http_code}" -X POST "$SPACE_URL/chat" \
      -H "Content-Type: application/json" \
      -d "{\"query\": \"test query $i\", \"max_results\": 1}")

    http_code=$(echo "$response" | grep "HTTP_CODE:" | cut -d':' -f2)
    body=$(echo "$response" | grep -v "HTTP_CODE:")

    echo "  Status: $http_code"
    echo "  Body: $(echo $body | head -c 100)..."
    echo ""

    sleep 2
done

echo "=========================================="
echo ""

echo "Test 2: Testing rate limit enforcement..."
echo "Sending 12 requests (should hit limit after 10)..."
echo ""

success_count=0
rate_limited_count=0

for i in {1..12}; do
    response=$(curl -s -w "\nHTTP_CODE:%{http_code}" -X POST "$SPACE_URL/chat" \
      -H "Content-Type: application/json" \
      -d "{\"query\": \"test $i\"}")

    http_code=$(echo "$response" | grep "HTTP_CODE:" | cut -d':' -f2)

    if [ "$http_code" == "429" ]; then
        rate_limited_count=$((rate_limited_count + 1))
        echo "Request $i: ⚠️  RATE LIMITED (429)"
    elif [ "$http_code" == "200" ] || [ "$http_code" == "500" ]; then
        success_count=$((success_count + 1))
        echo "Request $i: ✓ Processed ($http_code)"
    else
        echo "Request $i: ? Unexpected ($http_code)"
    fi
done

echo ""
echo "=========================================="
echo "Results:"
echo "  Processed: $success_count"
echo "  Rate Limited: $rate_limited_count"
echo ""

if [ $rate_limited_count -gt 0 ]; then
    echo "✅ Rate limiting is WORKING!"
    echo "   Requests were properly rate limited after reaching the threshold."
else
    echo "⚠️  Rate limiting may not be working as expected."
    echo "   All requests were processed without hitting the limit."
fi

echo ""
echo "=========================================="
echo ""

echo "Note:"
echo "- If you see mostly 500 errors with 429 in the body, this is"
echo "  from the external APIs (Cohere/OpenRouter), not your rate limiter."
echo "- If you see HTTP 429 responses directly, your rate limiter is working."
echo "- The Space may still be warm-starting or experiencing API limits."
echo ""
echo "To properly test, wait 1-2 minutes and run this script again."
