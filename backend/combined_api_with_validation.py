import os
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List
import logging
import requests
from urllib.parse import urlparse

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

def validate_qdrant_connection():
    """Validate that we can connect to the Qdrant instance."""
    qdrant_url = os.getenv("QDRANT_URL")
    if not qdrant_url:
        raise ValueError("QDRANT_URL environment variable is not set")
    
    # Parse the URL to extract host and port
    parsed = urlparse(qdrant_url)
    if not parsed.netloc:
        # If URL doesn't have protocol, add it
        if not qdrant_url.startswith("http"):
            qdrant_url = f"https://{qdrant_url}"
            parsed = urlparse(qdrant_url)
    
    # Try to make a simple request to verify connectivity
    health_url = f"{qdrant_url.strip('/')}/collections"
    headers = {
        "api-key": os.getenv("QDRANT_API_KEY")
    }
    
    try:
        response = requests.get(health_url, headers=headers, timeout=10)
        if response.status_code == 200:
            logger.info("Successfully connected to Qdrant")
            return True
        else:
            logger.error(f"Qdrant connection failed with status {response.status_code}")
            return False
    except requests.exceptions.ConnectionError:
        logger.error(f"Failed to connect to Qdrant at {qdrant_url}")
        return False
    except Exception as e:
        logger.error(f"Unexpected error connecting to Qdrant: {e}")
        return False

# Validate connection before initializing clients
if not validate_qdrant_connection():
    raise ConnectionError("Cannot connect to Qdrant. Please check your URL and API key.")

# Initialize clients
try:
    cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
        # Add timeout and retry settings
        timeout=30
    )
    logger.info("Clients initialized successfully")
except Exception as e:
    logger.error(f"Error initializing clients: {e}")
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
        # Create embedding for the query
        query_embedding = cohere_client.embed(
            texts=[request.query],
            model="embed-english-v3.0",
            input_type="search_query"
        ).embeddings[0]
        
        # Search in Qdrant
        search_results = qdrant_client.search(
            collection_name="book_docs",
            query_vector=query_embedding,
            limit=3,
            with_payload=True
        )
        
        # Extract text from results
        results = [result.payload["text"] for result in search_results]
        
        return QueryResponse(results=results)
    except Exception as e:
        logger.error(f"Error in query endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health")
async def health_check():
    try:
        # Verify Qdrant connection
        qdrant_client.get_collection("book_docs")
        return {"status": "healthy", "qdrant": "connected"}
    except Exception as e:
        logger.error(f"Health check failed: {e}")
        return {"status": "unhealthy", "error": str(e)}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)