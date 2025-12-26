from pydantic import BaseModel, EmailStr
from typing import Optional, List
from datetime import datetime
import json


# Background Information Schemas
class UserBackground(BaseModel):
    """Schema for user background information"""
    level: Optional[str] = None
    languages: Optional[List[str]] = []
    experience: Optional[str] = None
    tools: Optional[List[str]] = []


# User Schemas
class UserBase(BaseModel):
    email: EmailStr
    name: str


class UserCreate(UserBase):
    password: str
    software_background: Optional[UserBackground] = None
    hardware_background: Optional[UserBackground] = None


class UserUpdate(BaseModel):
    name: Optional[str] = None
    email: Optional[str] = None
    software_background: Optional[UserBackground] = None
    hardware_background: Optional[UserBackground] = None


class UserInDB(UserBase):
    id: int
    is_active: bool
    is_verified: bool
    software_background: Optional[UserBackground] = None
    hardware_background: Optional[UserBackground] = None
    created_at: datetime
    updated_at: Optional[datetime] = None

    class Config:
        from_attributes = True


class UserPublic(BaseModel):
    id: int
    email: EmailStr
    name: str
    is_verified: bool
    software_background: Optional[UserBackground] = None
    hardware_background: Optional[UserBackground] = None
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
class SignupRequest(BaseModel):
    email: EmailStr
    password: str
    name: str
    software_background: Optional[UserBackground] = None
    hardware_background: Optional[UserBackground] = None


class SigninRequest(BaseModel):
    email: EmailStr
    password: str


class AuthResponse(BaseModel):
    access_token: str
    token_type: str
    user: UserPublic