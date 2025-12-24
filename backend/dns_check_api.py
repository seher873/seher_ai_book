import os
import socket
from urllib.parse import urlparse
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

# Check the Qdrant URL
qdrant_url = os.getenv("QDRANT_URL")
logger.info(f"Qdrant URL: {qdrant_url}")

# Parse the URL to check its format
parsed_url = urlparse(qdrant_url)
logger.info(f"Parsed URL: scheme={parsed_url.scheme}, netloc={parsed_url.netloc}")

# Check if the domain is resolvable
try:
    domain = parsed_url.netloc.split(':')[0]  # Extract domain without port
    ip = socket.gethostbyname(domain)
    logger.info(f"Domain {domain} resolved to IP: {ip}")
except socket.gaierror as e:
    logger.error(f"Domain resolution failed: {e}")
    raise

# Initialize clients
try:
    cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
    logger.info("Cohere client initialized successfully")
except Exception as e:
    logger.error(f"Error initializing Cohere client: {e}")
    raise

try:
    qdrant_client = QdrantClient(
        url=qdrant_url,
        api_key=os.getenv("QDRANT_API_KEY"),
        https=True,
        timeout=30
    )
    
    # Test collection access
    collection_info = qdrant_client.get_collection("book_docs")
    logger.info(f"Collection 'book_docs' is accessible. Points count: {collection_info.points_count}")
    logger.info("Qdrant client initialized successfully")
except Exception as e:
    logger.error(f"Error with Qdrant connection: {e}")
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
        # Convert query to vector using Cohere
        query_response = cohere_client.embed(
            texts=[request.query],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        query_vector = query_response.embeddings[0]
        
        # Search Qdrant for top 3 most similar text chunks
        search_results = qdrant_client.search(
            collection_name="book_docs",
            query_vector=query_vector,
            limit=3,
            with_payload=True
        )
        
        # Extract text chunks from results
        results = [result.payload["text"] for result in search_results]
        
        return QueryResponse(results=results)
    except Exception as e:
        logger.error(f"Error in query endpoint: {e}")
        raise HTTPException(status_code=500, detail=f"Search failed: {str(e)}")

if __name__ == "__main__":
    import uvicorn
    logger.info("Starting server on http://0.0.0.0:8001")
    uvicorn.run(app, host="0.0.0.0", port=8001)