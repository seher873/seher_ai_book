#!/usr/bin/env python3
"""
Test OpenRouter API Connection
Tests the OpenRouter API key and model configuration
"""
import os
import sys
import json
from dotenv import load_dotenv
import openai

# Load environment variables
load_dotenv()

def test_openrouter_connection():
    """Test OpenRouter API connection with a simple request"""

    print("=" * 60)
    print("OpenRouter API Connection Test")
    print("=" * 60)

    # Check environment variables
    api_key = os.getenv("OPENROUTER_API_KEY")
    model = os.getenv("OPENROUTER_MODEL", "xiaomi/mimo-v2-flash-7b-instruct")

    print("\n1. Checking Configuration:")
    print(f"   API Key: {'✓ Set' if api_key else '✗ Missing'}")
    if api_key:
        # Show first and last 10 chars for security
        masked_key = api_key[:20] + "..." + api_key[-10:]
        print(f"   Key preview: {masked_key}")
    print(f"   Model: {model}")
    print(f"   Base URL: https://openrouter.ai/api/v1")

    if not api_key:
        print("\n✗ Error: OPENROUTER_API_KEY not found in .env file")
        return False

    # Initialize OpenAI client for OpenRouter
    print("\n2. Initializing OpenRouter Client...")
    try:
        client = openai.OpenAI(
            api_key=api_key,
            base_url="https://openrouter.ai/api/v1",
        )
        print("   ✓ Client initialized successfully")
    except Exception as e:
        print(f"   ✗ Failed to initialize client: {e}")
        return False

    # Test API connection with a simple request
    print("\n3. Testing API Connection...")
    try:
        response = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "user", "content": "Say 'OpenRouter API is working!' in one sentence."}
            ],
            max_tokens=100,
            temperature=0.7,
        )
        print("   ✓ API request successful")

        # Display response details
        print("\n4. Response Details:")
        print(f"   Model: {response.model}")
        print(f"   Stop Reason: {response.choices[0].finish_reason}")
        print(f"   Tokens Used: {response.usage.total_tokens}")
        print(f"   Input Tokens: {response.usage.prompt_tokens}")
        print(f"   Output Tokens: {response.usage.completion_tokens}")
        print(f"\n   Response: {response.choices[0].message.content}")

        return True

    except openai.AuthenticationError as e:
        print(f"   ✗ Authentication Error: Invalid API key or expired")
        print(f"   Details: {e}")
        return False
    except openai.APIError as e:
        print(f"   ✗ API Error: {e}")
        return False
    except Exception as e:
        print(f"   ✗ Unexpected Error: {e}")
        return False

def test_model_availability():
    """Test if the specific model is available"""

    print("\n" + "=" * 60)
    print("Model Availability Check")
    print("=" * 60)

    api_key = os.getenv("OPENROUTER_API_KEY")
    model = os.getenv("OPENROUTER_MODEL", "xiaomi/mimo-v2-flash-7b-instruct")

    if not api_key:
        print("\n✗ API Key not found, skipping model check")
        return False

    try:
        client = openai.OpenAI(
            api_key=api_key,
            base_url="https://openrouter.ai/api/v1",
        )

        # Get available models
        print(f"\nChecking if model '{model}' is available...")
        models = client.models.list()

        available_models = [m.id for m in models.data]

        if model in available_models:
            print(f"✓ Model '{model}' is available")
            return True
        else:
            print(f"✗ Model '{model}' not found in available models")
            print(f"\nAvailable models (first 10):")
            for m in available_models[:10]:
                print(f"  - {m}")
            return False

    except Exception as e:
        print(f"✗ Error checking models: {e}")
        return False

if __name__ == "__main__":
    # Run tests
    api_test_passed = test_openrouter_connection()
    model_test_passed = test_model_availability()

    # Summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    print(f"API Connection: {'✓ PASSED' if api_test_passed else '✗ FAILED'}")
    print(f"Model Availability: {'✓ PASSED' if model_test_passed else '✗ FAILED'}")

    if api_test_passed:
        print("\n✓ OpenRouter API is configured correctly and working!")
        sys.exit(0)
    else:
        print("\n✗ OpenRouter API test failed. Check your configuration.")
        sys.exit(1)
