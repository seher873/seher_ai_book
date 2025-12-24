import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

print("Environment variables check:")
print(f"QDRANT_URL: {os.getenv('QDRANT_URL')}")
print(f"QDRANT_API_KEY: {os.getenv('QDRANT_API_KEY')}")
print(f"COHERE_API_KEY: {os.getenv('COHERE_API_KEY')}")

# Test 1: Test Cohere connection
try:
    import cohere
    cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
    test_embedding = cohere_client.embed(
        texts=["test"],
        model="embed-english-v3.0",
        input_type="search_query"
    )
    print("✓ Cohere connection successful")
except Exception as e:
    print(f"✗ Cohere connection failed: {e}")

# Test 2: Test Qdrant connection
try:
    from qdrant_client import QdrantClient
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )
    collections = qdrant_client.get_collections()
    print("✓ Qdrant connection successful")
    print(f"Available collections: {[c.name for c in collections.collections]}")
except Exception as e:
    print(f"✗ Qdrant connection failed: {e}")

# Test 3: Test if required packages are available
try:
    import fastapi
    print("✓ FastAPI is available")
except ImportError:
    print("✗ FastAPI is not installed")

try:
    import uvicorn
    print("✓ Uvicorn is available")
except ImportError:
    print("✗ Uvicorn is not installed")

print("\nIf connections are successful, the issue might be with the server configuration.")