"""
BookCorpus model
Task: T034 [P] [US2] Create BookCorpus model in backend/rag_api/models/book_corpus.py

This represents the complete collection of Physical AI textbook content indexed 
as an immutable source for RAG operations (no overwrite, no mutation, no auto-editing).
"""
from typing import List, Optional
from pydantic import BaseModel, Field
from datetime import datetime


class BookCorpus(BaseModel):
    """
    Represents the complete collection of Physical AI textbook content 
    indexed as an immutable source for RAG operations.
    """
    id: str = Field(..., description="Unique identifier for the corpus")
    name: str = Field(..., description="Name of the textbook")
    version: str = Field(..., description="Version identifier")
    total_chunks: int = Field(ge=0, description="Number of knowledge chunks in corpus")
    indexing_date: datetime = Field(..., description="When the corpus was last indexed")
    checksum: str = Field(..., description="Integrity verification for the corpus")
    
    class Config:
        json_schema_extra = {
            "example": {
                "id": "physical_ai_textbook_v1",
                "name": "Physical AI Textbook",
                "version": "1.0.0",
                "total_chunks": 150,
                "indexing_date": "2024-01-01T00:00:00Z",
                "checksum": "a1b2c3d4e5f6..."
            }
        }