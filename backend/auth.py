"""
Authentication module for the Physical AI Textbook RAG Chatbot backend
Implements JWT-based authentication with user management
"""
from datetime import datetime, timedelta
from typing import Optional, Dict, Any
import jwt
import bcrypt
from fastapi import HTTPException, status, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel
from enum import Enum
import os

# Initialize security schemes
security = HTTPBearer()

# Secret key for JWT encoding/decoding (should be set in environment)
SECRET_KEY = os.getenv("JWT_SECRET_KEY", "your-default-secret-key-change-in-production")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

class UserRole(str, Enum):
    """User roles for authorization"""
    USER = "user"
    ADMIN = "admin"

class UserRegistration(BaseModel):
    """Model for user registration"""
    email: str
    password: str
    name: str
    software_background: Optional[Dict[str, Any]] = None
    hardware_background: Optional[Dict[str, Any]] = None

class UserLogin(BaseModel):
    """Model for user login"""
    email: str
    password: str

class User(BaseModel):
    """Model representing a user"""
    id: str
    email: str
    name: str
    role: UserRole = UserRole.USER
    software_background: Optional[Dict[str, Any]] = None
    hardware_background: Optional[Dict[str, Any]] = None
    created_at: datetime
    updated_at: datetime

class Token(BaseModel):
    """Model for JWT token"""
    access_token: str
    token_type: str

class TokenData(BaseModel):
    """Model for token data"""
    email: Optional[str] = None

class AuthService:
    """Authentication service class"""
    
    def __init__(self):
        # In a real application, this would connect to a database
        # For now, using in-memory storage for demonstration
        self.users = {}
    
    def hash_password(self, password: str) -> str:
        """Hash a password using bcrypt"""
        return bcrypt.hashpw(password.encode('utf-8'), bcrypt.gensalt()).decode('utf-8')
    
    def verify_password(self, plain_password: str, hashed_password: str) -> bool:
        """Verify a plain password against its hash"""
        return bcrypt.checkpw(plain_password.encode('utf-8'), hashed_password.encode('utf-8'))
    
    def create_access_token(self, data: dict, expires_delta: Optional[timedelta] = None) -> str:
        """Create a JWT access token"""
        to_encode = data.copy()
        if expires_delta:
            expire = datetime.utcnow() + expires_delta
        else:
            expire = datetime.utcnow() + timedelta(minutes=15)
        
        to_encode.update({"exp": expire})
        encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
        return encoded_jwt
    
    def decode_token(self, token: str) -> Optional[TokenData]:
        """Decode a JWT token and return the email"""
        try:
            payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
            email: str = payload.get("sub")
            if email is None:
                return None
            token_data = TokenData(email=email)
            return token_data
        except jwt.PyJWTError:
            return None
    
    def register_user(self, user_data: UserRegistration) -> User:
        """Register a new user"""
        # Check if user already exists
        if user_data.email in self.users:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="User with this email already exists"
            )
        
        # Hash the password
        hashed_password = self.hash_password(user_data.password)
        
        # Create user ID (in real app, use UUID)
        user_id = f"user_{len(self.users) + 1}"
        
        # Create user object
        user = User(
            id=user_id,
            email=user_data.email,
            name=user_data.name,
            role=UserRole.USER,
            software_background=user_data.software_background,
            hardware_background=user_data.hardware_background,
            created_at=datetime.utcnow(),
            updated_at=datetime.utcnow()
        )
        
        # Store user (in real app, save to database)
        self.users[user_data.email] = {
            "user": user,
            "hashed_password": hashed_password
        }
        
        return user
    
    def authenticate_user(self, email: str, password: str) -> Optional[User]:
        """Authenticate a user by email and password"""
        if email not in self.users:
            return None
        
        user_data = self.users[email]
        if not self.verify_password(password, user_data["hashed_password"]):
            return None
        
        return user_data["user"]
    
    def get_current_user(self, credentials: HTTPAuthorizationCredentials = Depends(security)) -> User:
        """Get the current authenticated user from the token"""
        token_data = self.decode_token(credentials.credentials)
        if token_data is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )
        
        if token_data.email not in self.users:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="User not found",
                headers={"WWW-Authenticate": "Bearer"},
            )
        
        return self.users[token_data.email]["user"]

# Create a global instance of the auth service
auth_service = AuthService()