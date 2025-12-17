"""Session management API endpoints for the RAG chatbot."""
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from datetime import datetime, timedelta
from ..models.session import ChatSession as SessionModel
from ..db.connection import get_db
from sqlalchemy.orm import Session
from ..config import Config
from ..utils.helpers import log_info
import uuid


router = APIRouter()


# Pydantic models for request/response
class CreateSessionRequest(BaseModel):
    pass  # For now, no specific request data needed


class CreateSessionResponse(BaseModel):
    session_token: str
    expires_at: str
    metadata: dict


class GetSessionResponse(BaseModel):
    session_token: str
    created_at: str
    last_activity_at: str
    expires_at: str
    is_active: bool


@router.post("/", response_model=CreateSessionResponse)
async def create_session(
    request: CreateSessionRequest,
    db: Session = Depends(get_db)
):
    """
    Create a new anonymous session.
    
    This endpoint creates a new chat session with a temporary session token
    that expires after a set period (defined in config).
    """
    log_info("Creating new session", "create_session")
    
    # Create a new session
    session = SessionModel(
        session_token=str(uuid.uuid4()),
        expires_at=datetime.utcnow() + timedelta(hours=Config.SESSION_EXPIRATION_HOURS),
        metadata={}
    )
    
    db.add(session)
    db.commit()
    db.refresh(session)
    
    return CreateSessionResponse(
        session_token=session.session_token,
        expires_at=session.expires_at.isoformat(),
        metadata=session.metadata
    )


@router.get("/{session_token}", response_model=GetSessionResponse)
async def get_session(
    session_token: str,
    db: Session = Depends(get_db)
):
    """
    Get information about a specific session.
    
    This endpoint retrieves session details including expiration status.
    """
    session = db.query(SessionModel).filter(SessionModel.session_token == session_token).first()
    
    if not session:
        raise HTTPException(status_code=404, detail="Session not found")
    
    current_time = datetime.utcnow()
    is_active = session.expires_at > current_time
    
    return GetSessionResponse(
        session_token=session.session_token,
        created_at=session.created_at.isoformat(),
        last_activity_at=session.last_activity_at.isoformat(),
        expires_at=session.expires_at.isoformat(),
        is_active=is_active
    )