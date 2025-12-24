import os
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List

# Load environment variables
load_dotenv()

# Initialize clients
try:
    cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
        https=True
    )
    # Verify collection exists
    qdrant_client.get_collection("book_docs")
    print("Successfully connected to Qdrant")
except Exception as e:
    print(f"Error initializing clients or connecting to collection: {e}")
    # Don't raise the exception immediately, allow the app to start
    qdrant_client = None

# FastAPI app
app = FastAPI()

class QueryRequest(BaseModel):
    query: str

class QueryResponse(BaseModel):
    results: List[str]

@app.get("/health")
async def health_check():
    if qdrant_client is None:
        return {"status": "degraded", "qdrant": "unavailable", "cohere": "available"}
    try:
        qdrant_client.get_collection("book_docs")
        return {"status": "healthy", "qdrant": "available", "cohere": "available"}
    except:
        return {"status": "degraded", "qdrant": "unavailable", "cohere": "available"}

@app.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    if qdrant_client is None:
        raise HTTPException(status_code=503, detail="Qdrant service is not available")

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
        raise HTTPException(status_code=500, detail=f"Search failed: {str(e)}")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)