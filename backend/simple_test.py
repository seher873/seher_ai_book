import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

# Load environment variables
load_dotenv()

print("Environment variables:")
print(f"QDRANT_URL: {os.getenv('QDRANT_URL', 'NOT SET')}")
print(f"QDRANT_API_KEY: {'SET' if os.getenv('QDRANT_API_KEY') else 'NOT SET'}")

# Test basic connection
try:
    # Initialize client with minimal settings
    client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
        https=True
    )
    
    # Test connection by getting collections
    collections = client.get_collections()
    collection_names = [c.name for c in collections.collections]
    
    print(f"✓ Successfully connected to Qdrant")
    print(f"Available collections: {collection_names}")
    
    # Check for our specific collection
    if "book_docs" in collection_names:
        print("✓ 'book_docs' collection exists")
        # Get collection info
        collection_info = client.get_collection("book_docs")
        print(f"Vectors in collection: {collection_info.points_count}")
    else:
        print("✗ 'book_docs' collection does not exist")
    
except Exception as e:
    print(f"✗ Connection failed: {e}")
    import traceback
    traceback.print_exc()