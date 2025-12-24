import os
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from fastapi import FastAPI
from pydantic import BaseModel
from typing import List
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load environment variables
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COHERE_API_KEY = os.getenv("COHERE_API_KEY")

logger.info(f"QDRANT_URL: {QDRANT_URL}")

# Initialize clients with specific configuration for cloud instance
try:
    cohere_client = cohere.Client(COHERE_API_KEY)
    logger.info("✓ Cohere client initialized")
except Exception as e:
    logger.error(f"✗ Cohere client initialization failed: {e}")
    raise

try:
    # Initialize Qdrant client with specific settings for cloud
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
        timeout=30
    )
    logger.info("✓ Qdrant client initialized")
except Exception as e:
    logger.error(f"✗ Qdrant client initialization failed: {e}")
    raise

# Create or verify collection
try:
    # Check if collection exists
    try:
        qdrant_client.get_collection("book_docs")
        logger.info("✓ Collection 'book_docs' already exists")
    except:
        # Create collection with cosine distance
        qdrant_client.recreate_collection(
            collection_name="book_docs",
            vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
        )
        logger.info("✓ Collection 'book_docs' created")
except Exception as e:
    logger.error(f"✗ Collection operation failed: {e}")
    raise

# Test that we can perform a simple operation
try:
    # Get collection info to verify everything is working
    collection_info = qdrant_client.get_collection("book_docs")
    logger.info(f"✓ Collection info retrieved, vectors count: {collection_info.points_count}")
except Exception as e:
    logger.error(f"✗ Collection info retrieval failed: {e}")
    raise

# FastAPI app
app = FastAPI(title="Qdrant RAG API", version="1.0.0")

class QueryRequest(BaseModel):
    query: str

class QueryResponse(BaseModel):
    results: List[str]

@app.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    try:
        logger.info(f"Received query: {request.query[:50]}...")
        
        # Generate embedding for the query
        query_response = cohere_client.embed(
            texts=[request.query],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        query_embedding = query_response.embeddings[0]
        logger.info("✓ Query embedding generated")
        
        # Search in Qdrant
        search_results = qdrant_client.search(
            collection_name="book_docs",
            query_vector=query_embedding,
            limit=3,
            with_payload=True
        )
        logger.info(f"✓ Found {len(search_results)} results")
        
        # Extract text from results
        results = []
        for result in search_results:
            if "text" in result.payload:
                results.append(result.payload["text"])
        
        logger.info("✓ Query processed successfully")
        return QueryResponse(results=results)
    except Exception as e:
        logger.error(f"✗ Error in query endpoint: {e}")
        raise

@app.get("/health")
async def health_check():
    return {"status": "healthy", "service": "Qdrant RAG API"}

if __name__ == "__main__":
    import uvicorn
    logger.info("Starting server on http://0.0.0.0:8001")
    uvicorn.run(app, host="0.0.0.0", port=8001)