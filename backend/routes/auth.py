from fastapi import APIRouter, Depends, HTTPException, status, BackgroundTasks
from fastapi.security import HTTPBearer
from sqlalchemy.orm import Session
from typing import Any
import uuid
from ..database import get_db
from ..models import User
from ..schemas import (
    LoginRequest, 
    LoginResponse, 
    SignupRequest, 
    SignupResponse,
    ForgotPasswordRequest,
    ResetPasswordRequest,
    UserPublic
)
from ..crud import create_user, authenticate_user, get_user_by_email
from ..utils import verify_password, get_password_hash, create_access_token
from ..config import settings
from ..email_service import send_verification_email, send_reset_password_email


router = APIRouter(prefix="/auth", tags=["Authentication"])

security = HTTPBearer()


@router.post("/signup", response_model=SignupResponse)
async def signup(
    user_data: SignupRequest, 
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
) -> Any:
    # Check if user already exists
    existing_user = get_user_by_email(db, user_data.email)
    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="A user with this email already exists"
        )
    
    # Create the user
    db_user = create_user(db, user_data)
    
    # Generate verification token
    verification_token = str(uuid.uuid4())
    db_user.verification_token = verification_token
    db.commit()
    
    # Send verification email in background
    background_tasks.add_task(
        send_verification_email, 
        user_data.email, 
        user_data.name, 
        verification_token
    )
    
    return {
        "user": UserPublic.from_orm(db_user),
        "message": "User created successfully. Please check your email to verify your account."
    }


@router.post("/login", response_model=LoginResponse)
def login(
    login_data: LoginRequest,
    db: Session = Depends(get_db)
) -> Any:
    user = authenticate_user(db, login_data.email, login_data.password)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    
    if not user.is_verified:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Email not verified. Please check your email for verification link.",
            headers={"WWW-Authenticate": "Bearer"},
        )
    
    # Create access token
    access_token = create_access_token(data={"sub": user.email})
    
    return {
        "access_token": access_token,
        "token_type": "bearer",
        "user": UserPublic.from_orm(user)
    }


@router.post("/logout")
def logout():
    # In a real implementation, you might want to invalidate the token
    # or add it to a blacklist until expiration
    return {"message": "Successfully logged out"}


@router.post("/forgot-password")
async def forgot_password(
    request: ForgotPasswordRequest,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
) -> Any:
    user = get_user_by_email(db, request.email)
    
    # Even if user doesn't exist, we return the same response to prevent email enumeration
    if not user:
        return {"message": "If an account with this email exists, a password reset link has been sent."}
    
    # Generate reset token
    reset_token = str(uuid.uuid4())
    user.verification_token = reset_token
    db.commit()
    
    # Send reset password email in background
    background_tasks.add_task(
        send_reset_password_email,
        user.email,
        user.name,
        reset_token
    )
    
    return {"message": "If an account with this email exists, a password reset link has been sent."}


@router.post("/reset-password")
def reset_password(
    request: ResetPasswordRequest,
    db: Session = Depends(get_db)
) -> Any:
    # Find user by reset token
    user = db.query(User).filter(User.verification_token == request.token).first()
    
    if not user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid or expired reset token"
        )
    
    # Hash the new password
    hashed_password = get_password_hash(request.new_password)
    user.hashed_password = hashed_password
    user.verification_token = None  # Clear the token after use
    db.commit()
    
    return {"message": "Password has been reset successfully"}


@router.get("/verify/{token}")
def verify_email(token: str, db: Session = Depends(get_db)) -> Any:
    # Find user by verification token
    user = db.query(User).filter(User.verification_token == token).first()
    
    if not user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid or expired verification token"
        )
    
    # Verify the user
    user.is_verified = True
    user.verification_token = None
    db.commit()
    
    return {"message": "Email has been verified successfully"}