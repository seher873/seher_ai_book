import os
from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    # Database settings
    DATABASE_URL: str = os.getenv("DATABASE_URL", "sqlite:///./test.db")
    
    # Auth settings
    JWT_SECRET_KEY: str = os.getenv("JWT_SECRET_KEY", "your-super-secret-key-change-in-production")
    JWT_ALGORITHM: str = os.getenv("JWT_ALGORITHM", "HS256")
    ACCESS_TOKEN_EXPIRE_MINUTES: int = int(os.getenv("ACCESS_TOKEN_EXPIRE_MINUTES", "30"))
    REFRESH_TOKEN_EXPIRE_DAYS: int = int(os.getenv("REFRESH_TOKEN_EXPIRE_DAYS", "7"))
    
    # Email settings
    MAIL_SERVER: str = os.getenv("MAIL_SERVER", "smtp.gmail.com")
    MAIL_PORT: int = int(os.getenv("MAIL_PORT", "587"))
    MAIL_USERNAME: str = os.getenv("MAIL_USERNAME")
    MAIL_PASSWORD: str = os.getenv("MAIL_PASSWORD")
    MAIL_FROM: str = os.getenv("MAIL_FROM", "noreply@yourapp.com")
    
    # Better Auth settings
    BETTER_AUTH_SECRET: str = os.getenv("BETTER_AUTH_SECRET", "your-development-secret-change-in-production")
    
    class Config:
        env_file = ".env"


settings = Settings()