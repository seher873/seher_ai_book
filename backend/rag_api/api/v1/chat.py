"""Chat API endpoints for the RAG chatbot."""
from fastapi import APIRouter, HTTPException, Depends
from typing import Optional
from pydantic import BaseModel, Field
import uuid
from datetime import datetime
from ..models.query import Query as QueryModel
from ..models.response import Response as ResponseModel
from ..models.session import ChatSession as SessionModel
from ..models.citation import Citation as CitationModel
from ..db.connection import get_db
from sqlalchemy.orm import Session, joinedload
from ..services.rag_service import rag_service
from ..services.content_filter import content_filter_service
from ..utils.validators import validate_query_content, validate_query_type
from ..utils.helpers import log_error, log_info
from ..config import Config


router = APIRouter()


# Pydantic models for request/response
class ChatQueryRequest(BaseModel):
    session_token: str = Field(..., description="Session token for the user session")
    query: str = Field(..., min_length=1, max_length=1000, description="User's question/query")
    query_context: Optional[dict] = Field(
        default={"type": "global"}, 
        description="Context for the query, including type and selected text if any"
    )


class ChatQueryResponse(BaseModel):
    response_id: str
    session_token: str
    query: str
    response: str
    citations: list[dict]
    timestamp: str


@router.post("/", response_model=ChatQueryResponse)
async def chat_endpoint(
    request: ChatQueryRequest,
    db: Session = Depends(get_db)
):
    """
    Submit a query to the RAG system and receive a response.
    
    This endpoint handles user queries and processes them through the RAG pipeline.
    It supports both global queries (searching the entire textbook corpus) and
    focused queries (searching specific selected text).
    """
    log_info(f"Received chat query for session: {request.session_token}", "chat_endpoint")
    
    # Validate query content
    is_valid, msg = validate_query_content(request.query)
    if not is_valid:
        raise HTTPException(status_code=422, detail=msg)
    
    # Validate query type
    query_type = request.query_context.get("type", "global")
    is_valid, msg = validate_query_type(query_type)
    if not is_valid:
        raise HTTPException(status_code=422, detail=msg)
    
    # Validate content with moderation service
    validation_result = content_filter_service.validate_content(request.query, "query")
    if not validation_result["is_approved"]:
        raise HTTPException(
            status_code=422, 
            detail=f"Content filtered as inappropriate: {validation_result['reason']}"
        )
    
    # Find the session
    session = db.query(SessionModel).filter(SessionModel.session_token == request.session_token).first()
    if not session:
        raise HTTPException(status_code=404, detail="Session not found or expired")
    
    # Check if session is still valid
    if session.expires_at < datetime.utcnow():
        raise HTTPException(status_code=401, detail="Session has expired")
    
    # Create a new query
    query = QueryModel(
        session_id=session.id,
        content=request.query,
        query_type=query_type,
        selected_text=request.query_context.get("selected_text"),
        timestamp=datetime.utcnow()
    )
    db.add(query)
    db.commit()
    db.refresh(query)
    
    try:
        # Process the query through RAG service
        result = await rag_service.process_query(
            query_content=query.content,
            session_token=query.session.session_token,
            query_type=query.query_type,
            selected_text=query.selected_text
        )
        
        # Validate the response content
        response_validation = content_filter_service.validate_content(result["response"], "response")
        if not response_validation["is_approved"]:
            raise HTTPException(
                status_code=500, 
                detail="Generated response was flagged by content filter"
            )
        
        # Create a response
        response = ResponseModel(
            query_id=query.id,
            content=result["response"],
            timestamp=datetime.utcnow()
        )
        db.add(response)
        db.commit()
        db.refresh(response)
        
        # Create citations for the response
        for citation_data in result["citations"]:
            citation = CitationModel(
                response_id=response.id,
                chapter_number=citation_data["chapter_number"],
                section_number=citation_data["section_number"],
                text_excerpt=citation_data["text_excerpt"],
                url=citation_data.get("url", "")
            )
            db.add(citation)
        
        # Commit all citations
        db.commit()
        
        return ChatQueryResponse(
            response_id=str(response.id),
            session_token=session.session_token,
            query=query.content,
            response=response.content,
            citations=result["citations"],
            timestamp=response.timestamp.isoformat()
        )
        
    except Exception as e:
        log_error(e, "chat_endpoint")
        db.rollback()
        raise HTTPException(status_code=500, detail="Internal server error during query processing")