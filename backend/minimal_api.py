import os
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

# Initialize clients with minimal configuration
try:
    cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
    logger.info("Cohere client initialized")
except Exception as e:
    logger.error(f"Error initializing Cohere client: {e}")
    raise

try:
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )
    logger.info("Qdrant client initialized")
except Exception as e:
    logger.error(f"Error initializing Qdrant client: {e}")
    raise

# Check if collection exists, create if not
try:
    collections = qdrant_client.get_collections()
    collection_names = [c.name for c in collections.collections]
    if "book_docs" not in collection_names:
        qdrant_client.recreate_collection(
            collection_name="book_docs",
            vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
        )
        logger.info("Collection 'book_docs' created")
    else:
        logger.info("Collection 'book_docs' already exists")
except Exception as e:
    logger.error(f"Error with collection: {e}")
    raise

# FastAPI app
app = FastAPI()

class QueryRequest(BaseModel):
    query: str

class QueryResponse(BaseModel):
    results: List[str]

@app.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    try:
        # Create embedding for the query
        response = cohere_client.embed(
            texts=[request.query],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        query_embedding = response.embeddings[0]
        
        # Search in Qdrant
        search_results = qdrant_client.search(
            collection_name="book_docs",
            query_vector=query_embedding,
            limit=3,
            with_payload=True
        )
        
        # Extract text from results
        results = []
        for result in search_results:
            if "text" in result.payload:
                results.append(result.payload["text"])
        
        return QueryResponse(results=results)
    except Exception as e:
        logger.error(f"Error in query endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)