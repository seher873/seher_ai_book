import os
from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    # Database settings
    DATABASE_URL: str = os.getenv("DATABASE_URL", "sqlite:///./test.db")

    # Auth settings
    JWT_SECRET_KEY: str = os.getenv("JWT_SECRET_KEY", "your-super-secret-key-change-in-production")
    JWT_ALGORITHM: str = os.getenv("JWT_ALGORITHM", "HS256")
    ACCESS_TOKEN_EXPIRE_MINUTES: int = int(os.getenv("ACCESS_TOKEN_EXPIRE_MINUTES", "30"))

    # Better Auth settings
    BETTER_AUTH_SECRET: str = os.getenv("BETTER_AUTH_SECRET", "your-development-secret-change-in-production")

    # Chatbot API Configuration
    CHATBOT_API_URL: str = os.getenv("CHATBOT_API_URL", "http://localhost:8000")
    REACT_APP_CHATBOT_API_URL: str = os.getenv("REACT_APP_CHATBOT_API_URL", "http://localhost:8000")

    class Config:
        env_file = ".env"


settings = Settings()