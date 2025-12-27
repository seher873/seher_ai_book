"""
Initialize the database by creating all tables
"""
import sys
sys.path.insert(0, '.')

from auth.database import engine, Base
from auth.models import User

def init_db():
    """Create all tables in the database"""
    print("Creating database tables...")
    Base.metadata.create_all(bind=engine)
    print("✓ Database tables created successfully!")

if __name__ == "__main__":
    init_db()
