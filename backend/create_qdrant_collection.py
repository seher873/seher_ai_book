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

def create_collection():
    qdrant_client.recreate_collection(
        collection_name="book_docs",
        vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
    )

def embed_text(texts):
    response = cohere_client.embed(
        texts=texts,
        model="embed-english-v3.0",
        input_type="search_document"
    )
    return response.embeddings

def upsert_points(chunks):
    embeddings = embed_text(chunks)
    points = []
    for i, chunk in enumerate(chunks):
        points.append(models.PointStruct(
            id=i,
            vector=embeddings[i],
            payload={"text": chunk}
        ))
    qdrant_client.upsert(
        collection_name="book_docs",
        points=points
    )

if __name__ == "__main__":
    create_collection()
    # Example chunks - replace with your actual document chunks
    example_chunks = [
        "Artificial intelligence is a wonderful field.",
        "Machine learning is a subset of AI.",
        "Deep learning uses neural networks."
    ]
    upsert_points(example_chunks)
    # Verify points were added
    count = qdrant_client.count(collection_name="book_docs")
    print(f"Points in collection: {count.count}")