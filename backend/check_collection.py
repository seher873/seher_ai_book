import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

# Load environment variables
load_dotenv()

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    https=True
)

# Get collection info
collection_info = qdrant_client.get_collection("book_docs")
print(f"Collection 'book_docs' points count: {collection_info.points_count}")

# Sample a few points if they exist
if collection_info.points_count > 0:
    # Get a few points to verify content
    points = qdrant_client.scroll(
        collection_name="book_docs",
        limit=3,
        with_payload=True,
        with_vectors=False
    )
    
    print("\nSample points from the collection:")
    for i, point in enumerate(points[0]):
        print(f"\nPoint {i+1}:")
        print(f"  ID: {point.id}")
        print(f"  Text: {point.payload.get('text', '')[:200]}...")  # First 200 chars
else:
    print("\nNo points found in the collection. You need to ingest documents first.")