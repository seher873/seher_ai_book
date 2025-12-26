from sqlalchemy.orm import Session
from sqlalchemy.exc import IntegrityError
from fastapi import HTTPException, status
from typing import Optional
from .models import User
from .schemas import UserCreate
from .utils import get_password_hash, verify_password


def get_user_by_email(db: Session, email: str) -> Optional[User]:
    """Get user by email"""
    return db.query(User).filter(User.email == email).first()


def get_user_by_id(db: Session, user_id: int) -> Optional[User]:
    """Get user by ID"""
    return db.query(User).filter(User.id == user_id).first()


def create_user(db: Session, user: UserCreate) -> User:
    """Create a new user with hashed password and background information"""
    # Hash the password
    hashed_password = get_password_hash(user.password)
    
    # Prepare background data as dictionaries
    software_bg_dict = user.software_background.dict() if user.software_background else None
    hardware_bg_dict = user.hardware_background.dict() if user.hardware_background else None
    
    # Create the user instance
    db_user = User(
        email=user.email,
        name=user.name,
        hashed_password=hashed_password,
        software_background=software_bg_dict,
        hardware_background=hardware_bg_dict
    )
    
    try:
        db.add(db_user)
        db.commit()
        db.refresh(db_user)
        return db_user
    except IntegrityError:
        db.rollback()
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="A user with this email already exists"
        )


def update_user(db: Session, user_id: int, user_update) -> User:
    """Update user information"""
    db_user = get_user_by_id(db, user_id)
    if not db_user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )
    
    # Update only provided fields
    if user_update.name is not None:
        db_user.name = user_update.name
    if user_update.email is not None:
        db_user.email = user_update.email
    if user_update.software_background is not None:
        db_user.software_background = user_update.software_background.dict()
    if user_update.hardware_background is not None:
        db_user.hardware_background = user_update.hardware_background.dict()
    
    db.commit()
    db.refresh(db_user)
    return db_user


def delete_user(db: Session, user_id: int) -> User:
    """Delete a user"""
    db_user = get_user_by_id(db, user_id)
    if not db_user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )
    
    db.delete(db_user)
    db.commit()
    return db_user


def authenticate_user(db: Session, email: str, password: str) -> Optional[User]:
    """Authenticate user with email and password"""
    user = get_user_by_email(db, email)
    if not user or not verify_password(password, user.hashed_password):
        return None
    return user