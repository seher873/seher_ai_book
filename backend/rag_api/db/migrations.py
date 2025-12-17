"""Database migrations framework for the RAG API."""
from sqlalchemy import create_engine, text
from sqlalchemy.exc import OperationalError
import os
from .connection import DATABASE_URL, Base
from ..models.session import ChatSession
from ..models.query import Query
from ..models.response import Response
from ..models.citation import Citation
from ..models.knowledge_chunk import KnowledgeChunk
from ..models.book_corpus import BookCorpus


def run_migrations():
    """Run database migrations to create all tables."""
    engine = create_engine(DATABASE_URL)
    
    # Create all tables defined in models
    Base.metadata.create_all(bind=engine)
    print("Database tables created successfully!")


def check_connection():
    """Check if the database connection is working."""
    engine = create_engine(DATABASE_URL)
    
    try:
        with engine.connect() as connection:
            # Execute a simple query to test connection
            result = connection.execute(text("SELECT 1"))
            return True
    except OperationalError as e:
        print(f"Database connection failed: {e}")
        return False


if __name__ == "__main__":
    print("Checking database connection...")
    if check_connection():
        print("Database connection successful. Running migrations...")
        run_migrations()
    else:
        print("Cannot connect to database. Please check your connection settings.")