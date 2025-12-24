import os
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List
import logging

# Set up logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

logger.info("Starting application initialization")

# Initialize clients with detailed error handling
try:
    logger.info("Initializing Cohere client...")
    cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
    logger.info("Cohere client initialized successfully")
except Exception as e:
    logger.error(f"Error initializing Cohere client: {e}")
    raise

try:
    logger.info("Initializing Qdrant client...")
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
        https=True,
        timeout=30
    )
    logger.info("Qdrant client initialized successfully")
    
    # Test collection access
    logger.info("Testing collection access...")
    collection_info = qdrant_client.get_collection("book_docs")
    logger.info(f"Collection 'book_docs' is accessible. Points count: {collection_info.points_count}")
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
        logger.info(f"Received query: {request.query[:50]}...")
        
        # Convert query to vector using Cohere
        query_response = cohere_client.embed(
            texts=[request.query],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        query_vector = query_response.embeddings[0]
        logger.info("Query vector created successfully")
        
        # Search Qdrant for top 3 most similar text chunks
        search_results = qdrant_client.search(
            collection_name="book_docs",
            query_vector=query_vector,
            limit=3,
            with_payload=True
        )
        logger.info(f"Search completed, found {len(search_results)} results")
        
        # Extract text chunks from results
        results = [result.payload["text"] for result in search_results]
        logger.info("Results extracted successfully")
        
        return QueryResponse(results=results)
    except Exception as e:
        logger.error(f"Error in query endpoint: {e}")
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"Search failed: {str(e)}")

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    logger.info("Starting server on http://0.0.0.0:8001")
    uvicorn.run(app, host="0.0.0.0", port=8001, log_level="debug")