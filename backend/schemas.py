from pydantic import BaseModel, EmailStr
from typing import Optional
from datetime import datetime


# User Schemas
class UserBase(BaseModel):
    email: EmailStr
    name: str


class UserCreate(UserBase):
    password: str


class UserUpdate(BaseModel):
    name: Optional[str] = None
    email: Optional[EmailStr] = None


class UserInDB(UserBase):
    id: int
    is_active: bool
    is_verified: bool
    created_at: datetime
    updated_at: Optional[datetime] = None

    class Config:
        from_attributes = True


class UserPublic(BaseModel):
    id: int
    email: EmailStr
    name: str
    is_verified: bool
    created_at: datetime

    class Config:
        from_attributes = True


# Token Schemas
class Token(BaseModel):
    access_token: str
    token_type: str


class TokenData(BaseModel):
    email: Optional[str] = None


# Authentication Schemas
class LoginRequest(BaseModel):
    email: EmailStr
    password: str


class LoginResponse(BaseModel):
    access_token: str
    token_type: str
    user: UserPublic


class SignupRequest(UserCreate):
    pass


class SignupResponse(BaseModel):
    user: UserPublic
    message: str = "User created successfully"


# Password Reset Schemas
class ForgotPasswordRequest(BaseModel):
    email: EmailStr


class ResetPasswordRequest(BaseModel):
    token: str
    new_password: str