import os
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Initialize clients
cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Create collection
qdrant_client.recreate_collection(
    collection_name="book_docs",
    vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
)

# Sample text chunks
text_chunks = [
    "Artificial intelligence is transforming the world.",
    "Machine learning is a subset of AI.",
    "Deep learning uses neural networks with multiple layers.",
    "Natural language processing helps machines understand human language.",
    "Computer vision enables machines to interpret visual information."
]

# Create embeddings
embeddings = cohere_client.embed(
    texts=text_chunks,
    model="embed-english-v3.0",
    input_type="search_document"
).embeddings

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
qdrant_client.upsert(
    collection_name="book_docs",
    points=points
)

# Print points count
count = qdrant_client.count(collection_name="book_docs")
print(f"Points in collection: {count.count}")