from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import List, Optional
import os
import cohere
import qdrant_client
from qdrant_client.http import models
import openai
from dotenv import load_dotenv

load_dotenv()

router = APIRouter()

# Initialize clients
co = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant = qdrant_client.QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Set OpenAI API key if using OpenAI for response generation
if os.getenv("OPENAI_API_KEY"):
    openai.api_key = os.getenv("OPENAI_API_KEY")

class ChatRequest(BaseModel):
    query: str
    max_results: Optional[int] = 5

class Source(BaseModel):
    title: str
    path: str
    score: float

class ChatResponse(BaseModel):
    response: str
    sources: List[Source]

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    try:
        # Generate embedding for the query
        response = co.embed(
            texts=[request.query],
            model="embed-multilingual-v3.0"  # Using Cohere's multilingual embedding model
        )
        query_embedding = response.embeddings[0]

        # Search in Qdrant for relevant content
        search_results = qdrant.search(
            collection_name="textbook_content",
            query_vector=query_embedding,
            limit=request.max_results
        )

        # Extract content from search results
        context_parts = []
        sources = []
        for result in search_results:
            content = result.payload.get("content", "")
            title = result.payload.get("title", "Unknown")
            path = result.payload.get("path", "Unknown")
            score = result.score
            
            context_parts.append(content)
            sources.append(Source(title=title, path=path, score=score))

        # Combine context
        context = "\n\n".join(context_parts)

        # Generate response using the context
        # Using a simple approach - in practice, you might want to use a more sophisticated method
        prompt = f"""
        You are an AI assistant for the Physical AI & Humanoid Robotics Textbook. 
        Use the following context to answer the user's question.
        If the context doesn't contain enough information to answer the question, say so.
        Be concise and accurate in your response.
        
        Context: {context}
        
        Question: {request.query}
        
        Answer:
        """

        # Using OpenAI for response generation
        client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        response = client.chat.completions.create(
            model=os.getenv("OPENROUTER_MODEL", "gpt-3.5-turbo"),
            messages=[
                {"role": "system", "content": "You are an AI assistant for the Physical AI & Humanoid Robotics Textbook. Answer questions based only on the provided context."},
                {"role": "user", "content": prompt}
            ],
            max_tokens=500,
            temperature=0.7
        )

        answer = response.choices[0].message.content.strip()

        return ChatResponse(response=answer, sources=sources)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")

class IngestRequest(BaseModel):
    documents: List[dict]  # Each document has 'title', 'content', 'path'

@router.post("/ingest")
async def ingest_content(request: IngestRequest):
    try:
        # Prepare documents for embedding
        texts = []
        payloads = []
        
        for doc in request.documents:
            texts.append(doc['content'])
            payloads.append({
                "title": doc['title'],
                "path": doc['path'],
                "content": doc['content']
            })

        # Generate embeddings
        response = co.embed(
            texts=texts,
            model="embed-multilingual-v3.0"
        )
        embeddings = response.embeddings

        # Prepare points for Qdrant
        points = []
        for i, (embedding, payload) in enumerate(zip(embeddings, payloads)):
            points.append(models.PointStruct(
                id=i,
                vector=embedding,
                payload=payload
            ))

        # Create or recreate the collection in Qdrant
        qdrant.recreate_collection(
            collection_name="textbook_content",
            vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
        )

        # Upload points to Qdrant
        qdrant.upload_points(
            collection_name="textbook_content",
            points=points
        )

        return {"success": True, "count": len(points)}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error ingesting content: {str(e)}")