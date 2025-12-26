from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer
from .utils import verify_token
from .schemas import UserInDB
from typing import Optional


# Security for protected routes
security = HTTPBearer()


def get_current_user(token: str = Depends(security)):
    """Dependency to get current authenticated user"""
    payload = verify_token(token.credentials)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid authentication credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # In a real implementation, you would fetch the user from the database
    # For now, we'll return a basic user object
    user = UserInDB(
        id=1,  # This would come from the token payload
        email=payload.get("sub"),
        name="Test User",  # This would also come from the token or database
        is_active=True,
        is_verified=True,
        software_background=None,
        hardware_background=None,
        created_at=None
    )
    return user