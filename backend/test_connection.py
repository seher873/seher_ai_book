import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

# Load environment variables
load_dotenv()

# Initialize Qdrant client
try:
    client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )
    
    # Try to access the collection
    client.get_collection("book_docs")
    print("✓ Successfully connected to Qdrant collection 'book_docs'")
    
except Exception as e:
    print(f"✗ Failed to connect to Qdrant collection 'book_docs': {e}")