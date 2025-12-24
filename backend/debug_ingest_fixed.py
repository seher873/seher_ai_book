import os
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
import logging

# Load environment variables from .env file
load_dotenv()

# Print environment variables for debugging
print("Environment variables:")
print(f"QDRANT_URL: {os.getenv('QDRANT_URL')}")
print(f"QDRANT_API_KEY: {os.getenv('QDRANT_API_KEY')}")
print(f"COHERE_API_KEY: {os.getenv('COHERE_API_KEY')}")

# Enable debug logging
logging.basicConfig(level=logging.DEBUG)

# Initialize clients
try:
    cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
    print("Cohere client initialized successfully")
except Exception as e:
    print(f"Error initializing Cohere client: {e}")
    exit(1)

try:
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    
    qdrant_client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key
    )
    print(f"Qdrant client initialized successfully with URL: {qdrant_url}")
except Exception as e:
    print(f"Error initializing Qdrant client: {e}")
    exit(1)

# Test connection to Qdrant
try:
    collections = qdrant_client.get_collections()
    print(f"Successfully connected to Qdrant. Available collections: {[c.name for c in collections.collections]}")
except Exception as e:
    print(f"Error connecting to Qdrant: {e}")
    exit(1)

# Create collection
try:
    qdrant_client.recreate_collection(
        collection_name="book_docs",
        vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
    )
    print("Collection 'book_docs' created successfully")
except Exception as e:
    print(f"Error creating collection: {e}")
    exit(1)

# Sample text chunks
text_chunks = [
    "Artificial intelligence is transforming the world.",
    "Machine learning is a subset of AI.",
    "Deep learning uses neural networks with multiple layers.",
    "Natural language processing helps machines understand human language.",
    "Computer vision enables machines to interpret visual information."
]

# Create embeddings
try:
    embeddings = cohere_client.embed(
        texts=text_chunks,
        model="embed-english-v3.0",
        input_type="search_document"
    ).embeddings
    print(f"Successfully created embeddings for {len(text_chunks)} text chunks")
except Exception as e:
    print(f"Error creating embeddings: {e}")
    exit(1)

# Prepare points
points = []
for i, (chunk, embedding) in enumerate(zip(text_chunks, embeddings)):
    point = models.PointStruct(
        id=i,
        vector=embedding,
        payload={"text": chunk}
    )
    points.append(point)

# Upsert points
try:
    qdrant_client.upsert(
        collection_name="book_docs",
        points=points
    )
    print("Successfully upserted points to Qdrant")
except Exception as e:
    print(f"Error upserting points: {e}")
    exit(1)

# Print points count
try:
    count = qdrant_client.count(collection_name="book_docs")
    print(f"Points in collection: {count.count}")
except Exception as e:
    print(f"Error getting count: {e}")
    exit(1)