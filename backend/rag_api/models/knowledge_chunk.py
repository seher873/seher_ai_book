"""KnowledgeChunk model representing segments of textbook content."""
from sqlalchemy import Column, String, DateTime, Text
from sqlalchemy.dialects.postgresql import UUID
from ..db.connection import Base
import uuid
from datetime import datetime


class KnowledgeChunk(Base):
    __tablename__ = "knowledge_chunks"

    id = Column(String, primary_key=True)  # Using string ID for better integration with vector DB
    chapter_number = Column(String(20), nullable=False)
    section_number = Column(String(20), nullable=False)
    content = Column(Text, nullable=False)
    embedding_vector = Column(String)  # Store as string representation of vector
    metadata = Column(String)  # JSON string for additional metadata
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        if not self.id:
            self.id = str(uuid.uuid4())
        if not self.created_at:
            self.created_at = datetime.utcnow()