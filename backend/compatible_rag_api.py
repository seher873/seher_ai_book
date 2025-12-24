import os
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import logging
import time

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

# Initialize clients with retry logic
max_retries = 3
retry_delay = 2

# Initialize Cohere client
for attempt in range(max_retries):
    try:
        logger.info(f"Initializing Cohere client (attempt {attempt + 1}/{max_retries})...")
        cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
        logger.info("Cohere client initialized successfully")
        break
    except Exception as e:
        logger.error(f"Cohere client initialization failed: {e}")
        if attempt == max_retries - 1:
            raise
        time.sleep(retry_delay)

# Initialize Qdrant client with retry logic
for attempt in range(max_retries):
    try:
        logger.info(f"Initializing Qdrant client (attempt {attempt + 1}/{max_retries})...")
        qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY"),
            https=True,
            timeout=30
        )
        
        # Test collection access
        logger.info("Testing collection access...")
        collection_info = qdrant_client.get_collection("book_docs")
        logger.info(f"Collection 'book_docs' is accessible. Points count: {collection_info.points_count}")
        logger.info("Qdrant client initialized successfully")
        break
    except Exception as e:
        logger.error(f"Qdrant client initialization failed: {e}")
        if attempt == max_retries - 1:
            raise
        time.sleep(retry_delay)

# FastAPI app
app = FastAPI()

class QueryRequest(BaseModel):
    query: str
    max_results: Optional[int] = 5

class Source(BaseModel):
    id: str
    title: Optional[str] = None
    path: Optional[str] = None
    score: Optional[float] = None

class ChatResponse(BaseModel):
    query: str
    response: str
    sources: List[Source]

class QueryResponse(BaseModel):
    results: List[str]

@app.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    try:
        logger.info(f"Received query: {request.query[:50]}...")
        
        # Convert query to vector using Cohere with retry logic
        for attempt in range(max_retries):
            try:
                query_response = cohere_client.embed(
                    texts=[request.query],
                    model="embed-english-v3.0",
                    input_type="search_query"
                )
                query_vector = query_response.embeddings[0]
                logger.info("Query vector created successfully")
                break
            except Exception as e:
                logger.error(f"Cohere embedding failed: {e}")
                if attempt == max_retries - 1:
                    raise
                time.sleep(retry_delay)
        
        # Search Qdrant for most similar text chunks with retry logic
        for attempt in range(max_retries):
            try:
                search_results = qdrant_client.search(
                    collection_name="book_docs",
                    query_vector=query_vector,
                    limit=request.max_results,
                    with_payload=True
                )
                logger.info(f"Search completed, found {len(search_results)} results")
                break
            except Exception as e:
                logger.error(f"Qdrant search failed: {e}")
                if attempt == max_retries - 1:
                    raise
                time.sleep(retry_delay)
        
        # Extract text chunks from results
        results = [result.payload["text"] for result in search_results]
        logger.info("Results extracted successfully")
        
        return QueryResponse(results=results)
    except Exception as e:
        logger.error(f"Error in query endpoint: {e}")
        raise HTTPException(status_code=500, detail=f"Search failed: {str(e)}")

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: QueryRequest):
    try:
        logger.info(f"Received chat query: {request.query[:50]}...")
        
        # Convert query to vector using Cohere with retry logic
        for attempt in range(max_retries):
            try:
                query_response = cohere_client.embed(
                    texts=[request.query],
                    model="embed-english-v3.0",
                    input_type="search_query"
                )
                query_vector = query_response.embeddings[0]
                logger.info("Query vector created successfully")
                break
            except Exception as e:
                logger.error(f"Cohere embedding failed: {e}")
                if attempt == max_retries - 1:
                    raise
                time.sleep(retry_delay)
        
        # Search Qdrant for most similar text chunks with retry logic
        for attempt in range(max_retries):
            try:
                search_results = qdrant_client.search(
                    collection_name="book_docs",
                    query_vector=query_vector,
                    limit=request.max_results,
                    with_payload=True
                )
                logger.info(f"Search completed, found {len(search_results)} results")
                break
            except Exception as e:
                logger.error(f"Qdrant search failed: {e}")
                if attempt == max_retries - 1:
                    raise
                time.sleep(retry_delay)
        
        # Extract text chunks and create sources
        results = []
        sources = []
        for i, result in enumerate(search_results):
            text = result.payload["text"]
            results.append(text)
            
            # Create source object
            source = Source(
                id=result.id,
                title=result.payload.get("title", ""),
                path=result.payload.get("path", ""),
                score=result.score
            )
            sources.append(source)
        
        logger.info("Chat response prepared successfully")
        
        # For now, return the context as the response
        # In a real implementation, you would send this to a language model
        response_text = "\n\n".join(results[:3])  # Return top 3 results
        
        return ChatResponse(
            query=request.query,
            response=response_text,
            sources=sources
        )
    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail=f"Chat failed: {str(e)}")

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    logger.info("Starting server on http://0.0.0.0:8002")
    uvicorn.run(app, host="0.0.0.0", port=8002)