#!/usr/bin/env python3
"""
Comprehensive Backend API Test
Tests the full RAG chatbot backend with the corrected OpenRouter model
"""
import requests
import json
import time
import sys
from datetime import datetime

def test_health_check():
    """Test the health check endpoint"""
    print("\n" + "=" * 60)
    print("Testing Health Check Endpoint")
    print("=" * 60)

    try:
        response = requests.get("http://localhost:8000/health", timeout=5)
        print(f"Status Code: {response.status_code}")
        if response.status_code == 200:
            data = response.json()
            print("✓ Health check passed")
            print(f"Response: {json.dumps(data, indent=2)}")
            return True
        else:
            print(f"✗ Health check failed: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Connection error: {e}")
        return False

def test_chat_endpoint():
    """Test the chat endpoint with a simple query"""
    print("\n" + "=" * 60)
    print("Testing Chat Endpoint")
    print("=" * 60)

    payload = {
        "query": "What is Physical AI?",
        "max_results": 3
    }

    print(f"Query: {payload['query']}")
    print(f"Max Results: {payload['max_results']}")

    try:
        print("\nSending request to /chat endpoint...")
        start_time = time.time()

        response = requests.post(
            "http://localhost:8000/chat",
            json=payload,
            timeout=60
        )

        elapsed_time = time.time() - start_time

        print(f"Status Code: {response.status_code}")
        print(f"Response Time: {elapsed_time:.2f} seconds")

        if response.status_code == 200:
            data = response.json()
            print("\n✓ Chat request successful")

            print("\nResponse Content:")
            print(f"  Response: {data.get('response', 'N/A')[:200]}...")

            sources = data.get('sources', [])
            print(f"\nSources Used: {len(sources)}")
            for i, src in enumerate(sources, 1):
                print(f"  {i}. {src.get('title', 'Unknown')} (Score: {src.get('score', 0):.4f})")

            return True
        else:
            print(f"\n✗ Chat request failed")
            print(f"Response: {response.text}")

            # Check for rate limiting
            if response.status_code == 429:
                print("\n⚠ Rate limit error detected (429)")
                print("Note: OpenRouter API may have rate limits on free models")

            return False

    except requests.Timeout:
        print(f"✗ Request timeout after 60 seconds")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

def test_direct_openrouter():
    """Test OpenRouter API directly to verify model"""
    print("\n" + "=" * 60)
    print("Testing OpenRouter Model Directly")
    print("=" * 60)

    try:
        import openai
        from dotenv import load_dotenv
        import os

        load_dotenv()

        api_key = os.getenv("OPENROUTER_API_KEY")
        model = os.getenv("OPENROUTER_MODEL")

        if not api_key:
            print("✗ OPENROUTER_API_KEY not found")
            return False

        print(f"Model: {model}")
        print(f"API Key: {api_key[:20]}...{api_key[-10:]}")

        client = openai.OpenAI(
            api_key=api_key,
            base_url="https://openrouter.ai/api/v1",
        )

        print("\nSending test request...")
        start_time = time.time()

        response = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "user", "content": "Write a one-sentence summary of what Physical AI is."}
            ],
            max_tokens=150,
            temperature=0.7,
        )

        elapsed_time = time.time() - start_time

        print(f"✓ OpenRouter request successful")
        print(f"Response Time: {elapsed_time:.2f} seconds")
        print(f"Tokens Used: {response.usage.total_tokens}")
        print(f"Output: {response.choices[0].message.content}")

        return True

    except Exception as e:
        print(f"✗ Error: {e}")
        return False

def main():
    print("\n" + "=" * 60)
    print(f"Backend API Test Suite - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 60)

    print("\n⚠ Note: Make sure the backend server is running on port 8000")
    print("   Run: python main.py")

    # Test direct OpenRouter first
    openrouter_ok = test_direct_openrouter()

    # Test health check
    health_ok = test_health_check()

    # Test chat endpoint
    chat_ok = test_chat_endpoint()

    # Summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    print(f"OpenRouter Connection: {'✓ PASSED' if openrouter_ok else '✗ FAILED'}")
    print(f"Health Check: {'✓ PASSED' if health_ok else '✗ FAILED'}")
    print(f"Chat Endpoint: {'✓ PASSED' if chat_ok else '✗ FAILED'}")

    if openrouter_ok and health_ok and chat_ok:
        print("\n✓ All tests passed! Backend is working correctly.")
        return 0
    else:
        print("\n✗ Some tests failed. Check the output above for details.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
