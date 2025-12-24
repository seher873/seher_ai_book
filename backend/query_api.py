import os
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from fastapi import FastAPI
from pydantic import BaseModel
from typing import List

# Load environment variables
load_dotenv()

# Initialize clients
cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# FastAPI app
app = FastAPI()

class QueryRequest(BaseModel):
    query: str

class QueryResponse(BaseModel):
    results: List[str]

@app.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
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

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)