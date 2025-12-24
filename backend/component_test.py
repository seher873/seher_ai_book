import os
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models

print("Testing individual components...")

# Test 1: Check environment variables
print(f"QDRANT_URL: {os.getenv('QDRANT_URL', 'NOT SET')}")
print(f"QDRANT_API_KEY: {os.getenv('QDRANT_API_KEY', 'NOT SET')}")
print(f"COHERE_API_KEY: {os.getenv('COHERE_API_KEY', 'NOT SET')}")

# Test 2: Initialize Cohere client
try:
    cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
    print("✓ Cohere client initialized successfully")
    
    # Test embedding
    test_embedding = cohere_client.embed(
        texts=["test"],
        model="embed-english-v3.0",
        input_type="search_query"
    )
    print("✓ Cohere embedding test successful")
except Exception as e:
    print(f"✗ Cohere test failed: {e}")

# Test 3: Initialize Qdrant client
try:
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )
    print("✓ Qdrant client initialized successfully")
    
    # Test collection access
    collections = qdrant_client.get_collections()
    print(f"✓ Qdrant connection successful. Collections: {[c.name for c in collections.collections]}")
except Exception as e:
    print(f"✗ Qdrant test failed: {e}")

print("\nIf all tests passed, try running the server again:")
print("cd /mnt/c/Users/user/Desktop/my_ai_book/backend")
print("uvicorn minimal_api:app --host 0.0.0.0 --port 8001")