"""Query model representing a user's input to the chatbot with conversation context."""
from sqlalchemy import Column, String, DateTime, Text, ForeignKey, JSON
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
import uuid
from datetime import datetime
from ..db.connection import Base


class Query(Base):
    __tablename__ = "queries"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), ForeignKey("chat_sessions.id"), nullable=False)
    content = Column(Text, nullable=False)
    query_type = Column(String(20), nullable=False)  # 'global' or 'focused'
    selected_text = Column(Text)  # For focused queries
    timestamp = Column(DateTime, default=datetime.utcnow, nullable=False)
    source_chunk_ids = Column(String)  # Comma-separated list of chunk IDs used for response
    conversation_context = Column(JSON)  # Stores conversation history for context awareness

    # Relationships
    session = relationship("ChatSession", back_populates="queries")
    response = relationship("Response", back_populates="query", uselist=False, cascade="all, delete-orphan")

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        if not self.id:
            self.id = uuid.uuid4()
        if not self.timestamp:
            self.timestamp = datetime.utcnow()
        if self.query_type not in ["global", "focused"]:
            raise ValueError("query_type must be either 'global' or 'focused'")
        if self.conversation_context is None:
            self.conversation_context = []