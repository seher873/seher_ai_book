"""
Authentication module using better-auth for FastAPI
Implements signup and signin functionality with TDD approach
"""
from fastapi import FastAPI, Depends, HTTPException, status # type: ignore
from fastapi.security import HTTPBearer # type: ignore
from sqlalchemy.orm import Session # type: ignore
from typing import Optional
from .database import get_db
from .schemas import SignupRequest, SigninRequest, AuthResponse, UserPublic
from .services import create_user, authenticate_user
from .utils import create_access_token
from .config import settings


# Security for protected routes
security = HTTPBearer()


def get_current_user():
    """Dependency to get current authenticated user"""
    def current_user(token: str = Depends(security)):
        # This function would verify the token and return user
        # Implementation would depend on how tokens are stored/verified
        from .utils import verify_token
        payload = verify_token(token)
        if payload is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid authentication credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )
        return payload
    return current_user


# Initialize auth routes
def init_auth_routes(app: FastAPI):
    """Initialize authentication routes with the FastAPI app"""

    @app.post("/auth/signup", response_model=AuthResponse)
    async def signup(
        signup_data: SignupRequest,
        db: Session = Depends(get_db)
    ):
        """Signup endpoint with background information"""
        try:
            # Create user using service layer
            db_user = create_user(db, signup_data)

            # Create access token
            access_token = create_access_token(data={"sub": db_user.email})

            # Prepare response using UserPublic schema
            user_response = UserPublic.model_validate(db_user)

            return AuthResponse(
                access_token=access_token,
                token_type="bearer",
                user=user_response
            )
        except HTTPException:
            # Re-raise HTTP exceptions
            raise
        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Signup failed: {str(e)}"
            )

    @app.post("/auth/signin", response_model=AuthResponse)
    async def signin(
        signin_data: SigninRequest,
        db: Session = Depends(get_db)
    ):
        """Signin endpoint"""
        try:
            # Authenticate user using service layer
            user = authenticate_user(
                db,
                signin_data.email,
                signin_data.password
            )

            if not user:
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="Incorrect email or password",
                    headers={"WWW-Authenticate": "Bearer"},
                )

            if not user.is_active:
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="Inactive user",
                    headers={"WWW-Authenticate": "Bearer"},
                )

            # Create access token
            access_token = create_access_token(data={"sub": user.email})

            # Prepare response using UserPublic schema
            user_response = UserPublic.model_validate(user)

            return AuthResponse(
                access_token=access_token,
                token_type="bearer",
                user=user_response
            )
        except HTTPException:
            # Re-raise HTTP exceptions
            raise
        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail=f"Signin failed: {str(e)}"
            )