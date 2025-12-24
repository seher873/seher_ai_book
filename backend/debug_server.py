import os
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from fastapi import FastAPI
from pydantic import BaseModel
from typing import List
import logging

# Set up detailed logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Load environment variables
logger.info(f"QDRANT_URL: {os.getenv('QDRANT_URL')}")
logger.info(f"QDRANT_API_KEY: {os.getenv('QDRANT_API_KEY')}")
logger.info(f"COHERE_API_KEY: {os.getenv('COHERE_API_KEY')}")

# Initialize clients
try:
    cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
    logger.info("Cohere client initialized successfully")
except Exception as e:
    logger.error(f"Error initializing Cohere client: {e}")
    raise

try:
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )
    logger.info("Qdrant client initialized successfully")
except Exception as e:
    logger.error(f"Error initializing Qdrant client: {e}")
    raise

# Create collection if not exists
try:
    qdrant_client.get_collection("book_docs")
    logger.info("Collection 'book_docs' already exists")
except:
    qdrant_client.recreate_collection(
        collection_name="book_docs",
        vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
    )
    logger.info("Collection 'book_docs' created")

# FastAPI app
app = FastAPI()

class QueryRequest(BaseModel):
    query: str

class QueryResponse(BaseModel):
    results: List[str]

@app.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    try:
        logger.info(f"Received query: {request.query}")
        
        # Create embedding for the query
        query_embedding = cohere_client.embed(
            texts=[request.query],
            model="embed-english-v3.0",
            input_type="search_query"
        ).embeddings[0]
        
        logger.info("Created query embedding")
        
        # Search in Qdrant
        search_results = qdrant_client.search(
            collection_name="book_docs",
            query_vector=query_embedding,
            limit=3,
            with_payload=True
        )
        
        logger.info(f"Found {len(search_results)} results from Qdrant")
        
        # Extract text from results
        results = [result.payload["text"] for result in search_results]
        
        logger.info(f"Returning results: {results}")
        return QueryResponse(results=results)
    except Exception as e:
        logger.error(f"Error in query endpoint: {e}")
        raise

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    logger.info("Starting server...")
    uvicorn.run(app, host="0.0.0.0", port=8001, log_level="debug")