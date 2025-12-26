"""
Authentication module for the Physical AI Textbook RAG Chatbot backend
Implements JWT-based authentication with user management using better-auth
"""
from .main import init_auth_routes
from .services import create_user, authenticate_user
from .utils import get_password_hash, verify_password, create_access_token, verify_token
from .schemas import SignupRequest, SigninRequest, AuthResponse, UserBackground
from .models import User

__all__ = [
    "init_auth_routes",
    "create_user", 
    "authenticate_user",
    "get_password_hash",
    "verify_password",
    "create_access_token",
    "verify_token",
    "SignupRequest",
    "SigninRequest", 
    "AuthResponse",
    "UserBackground",
    "User"
]