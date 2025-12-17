"""ChatSession model representing a single conversation between user and chatbot with conversation history."""
from sqlalchemy import Column, String, DateTime, JSON
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.mutable import MutableDict
import uuid
from datetime import datetime, timedelta
from ..db.connection import Base


class ChatSession(Base):
    __tablename__ = "chat_sessions"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_token = Column(String, unique=True, nullable=False, index=True)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    last_activity_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)
    expires_at = Column(DateTime, nullable=False)
    metadata = Column(MutableDict.as_mutable(JSON), default={})
    conversation_history = Column(MutableDict.as_mutable(JSON), default=list)

    # Relationships
    queries = relationship("Query", back_populates="session", cascade="all, delete-orphan")

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        if not self.id:
            self.id = uuid.uuid4()
        if not self.session_token:
            self.session_token = str(uuid.uuid4())
        if not self.expires_at:
            # Default to 24 hours from creation
            self.expires_at = datetime.utcnow() + timedelta(hours=24)
        if self.conversation_history is None:
            self.conversation_history = []

    def add_to_conversation_history(self, query_content: str, response_content: str, citations: list = None):
        """Add a query-response pair to the conversation history."""
        if citations is None:
            citations = []

        entry = {
            "query": query_content,
            "response": response_content,
            "citations": citations,
            "timestamp": datetime.utcnow().isoformat()
        }

        self.conversation_history.append(entry)

        # Limit history to prevent it from growing indefinitely
        # Keep only the last 20 exchanges (adjust as needed)
        if len(self.conversation_history) > 20:
            self.conversation_history = self.conversation_history[-20:]

    def get_recent_conversation_context(self, limit: int = 5) -> list:
        """Get the most recent conversation exchanges for context."""
        if not self.conversation_history:
            return []

        return self.conversation_history[-limit:]