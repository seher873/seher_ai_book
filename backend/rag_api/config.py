"""Configuration management for API keys and settings."""
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class Config:
    """Configuration class to manage application settings."""
    
    # API Keys
    OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    QDRANT_HOST = os.getenv("QDRANT_HOST", "localhost")
    
    # Database
    DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://user:password@localhost/rag_chatbot")
    
    # Security
    SECRET_KEY = os.getenv("SECRET_KEY", "dev-secret-key-change-in-production")
    ALGORITHM = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES = 30
    
    # Application settings
    APP_NAME = "RAG Chatbot API"
    APP_VERSION = "1.0.0"
    DEBUG = os.getenv("DEBUG", "False").lower() == "true"
    
    # RAG settings
    MAX_QUERY_LENGTH = 1000  # Maximum length of user query
    MAX_RESPONSE_TOKENS = 1500  # Maximum tokens in response
    EMBEDDING_DIMENSION = 1536  # Dimension of OpenAI embeddings (text-embedding-3-small)
    
    # Session settings
    SESSION_EXPIRATION_HOURS = 24  # How long a session token is valid
    
    # Validation
    @classmethod
    def validate(cls):
        """Validate that all required configuration values are set."""
        errors = []
        
        if not cls.OPENAI_API_KEY:
            errors.append("OPENAI_API_KEY is not set")
        
        if not cls.DATABASE_URL:
            errors.append("DATABASE_URL is not set")
        
        if not cls.SECRET_KEY or cls.SECRET_KEY == "dev-secret-key-change-in-production":
            errors.append("SECRET_KEY is not set or is using default dev value")
        
        return errors

# Validate configuration on import
config_errors = Config.validate()
if config_errors:
    raise ValueError(f"Configuration errors: {', '.join(config_errors)}")