"""
RAG Chatbot Backend for Physical AI & Humanoid Robotics Textbook
"""
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import os
import cohere
import qdrant_client
from qdrant_client.http import models
import openai
from dotenv import load_dotenv
import logging

# Load environment variablesrun 

load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize services
if not os.getenv("COHERE_API_KEY"):
    raise RuntimeError("COHERE_API_KEY missing")
if not os.getenv("OPENROUTER_API_KEY"):
    raise RuntimeError("OPENROUTER_API_KEY missing")
if not os.getenv("QDRANT_URL") or not os.getenv("QDRANT_API_KEY"):
    raise RuntimeError("QDRANT_URL and QDRANT_API_KEY missing")

cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
openai_client = openai.OpenAI(
    api_key=os.getenv("OPENROUTER_API_KEY"),
    base_url="https://openrouter.ai/api/v1",
)
qdrant_client = qdrant_client.QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# RAG Architecture Diagram:
# 
# User Query
#     ↓
# [Embedding Generation with Cohere]
#     ↓
# [Vector Search in Qdrant]
#     ↓
# [Context Retrieval]
#     ↓
# [Gemini 1.5 Flash Generation]
#     ↓
# RAG Response

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI Textbook RAG Chatbot",
    description="RAG system for the Physical AI & Humanoid Robotics textbook",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Constants
COLLECTION_NAME = "seher_robotic_book_netlify_app"
EMBEDDING_MODEL = "embed-english-v3.0"
GENERATION_MODEL = os.getenv("OPENROUTER_MODEL", "xiaomi/mimo-v2-flash")

class ChatRequest(BaseModel):
    query: str
    max_results: Optional[int] = 5

class ChatResponse(BaseModel):
    query: str
    response: str
    sources: List[dict]

class Document(BaseModel):
    id: str
    content: str
    title: Optional[str] = None
    path: Optional[str] = None

@app.get("/")
def read_root():
    return {"message": "Physical AI Textbook RAG Chatbot Backend"}

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    try:
        # Generate embedding for the query
        query_response = cohere_client.embed(
            texts=[request.query],
            model=EMBEDDING_MODEL,
            input_type="search_query"
        )
        query_embedding = query_response.embeddings[0]
        
        # Search for relevant documents in Qdrant
        search_results = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=request.max_results,
            with_payload=True
        )
        
        # Extract content from search results
        relevant_contents = []
        sources = []
        
        for result in search_results:
            content = result.payload.get("content", "")
            relevant_contents.append(content)
            
            sources.append({
                "id": result.id,
                "title": result.payload.get("title", "Unknown"),
                "path": result.payload.get("path", "Unknown"),
                "score": result.score
            })
        
        # If no relevant content found, return appropriate response
        if not relevant_contents:
            return ChatResponse(
                query=request.query,
                response="Not found in the book",
                sources=[]
            )
        
        # Combine context for generation
        context = "\n\n".join(relevant_contents)
        
        # Create prompt for response generation
        prompt = f"""
        Based on the provided context from the Physical AI & Humanoid Robotics textbook, 
        please answer the user's question. If the information is not available in the 
        context, respond with "Not found in the book".
        
        Context: {context}
        
        Question: {request.query}
        
        Answer:
        """
        
        # Generate response using OpenRouter
        response = openai_client.chat.completions.create(
            model=GENERATION_MODEL,
            messages=[
                {"role": "system", "content": "You are an AI assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based on the provided context. If the information is not available in the context, respond with 'Not found in the book'."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.7,
            max_tokens=1000
        )

        # Extract the response text
        response_text = response.choices[0].message.content

        # Check if response indicates information is not in the book
        if "Not found in the book" in response_text or response_text.strip() == "":
            return ChatResponse(
                query=request.query,
                response="Not found in the book",
                sources=[]
            )

        return ChatResponse(
            query=request.query,
            response=response_text,
            sources=sources
        )
        
    except Exception as e:
        logger.error(f"Error processing chat request: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/ingest")
async def ingest_documents(documents: List[Document]):
    try:
        # Prepare documents for embedding
        texts_to_embed = [doc.content for doc in documents]
        
        # Generate embeddings using Cohere
        embeddings_response = cohere_client.embed(
            texts=texts_to_embed,
            model=EMBEDDING_MODEL,
            input_type="search_document"
        )
        
        embeddings = embeddings_response.embeddings
        
        # Prepare points for Qdrant
        points = []
        for i, doc in enumerate(documents):
            points.append(models.PointStruct(
                id=doc.id,
                vector=embeddings[i],
                payload={
                    "content": doc.content,
                    "title": doc.title or "",
                    "path": doc.path or ""
                }
            ))
        
        # Upsert points to Qdrant
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )
        
        return {"message": f"Successfully ingested {len(documents)} documents"}
        
    except Exception as e:
        logger.error(f"Error ingesting documents: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health")
async def health_check():
    return {"status": "healthy", "service": "RAG Chatbot Backend"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)