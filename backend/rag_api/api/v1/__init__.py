"""API v1 routing and middleware setup."""
from fastapi import APIRouter
from . import chat, sessions, textbook

# Create the main API router for version 1
api_router = APIRouter()

# Include individual route modules
api_router.include_router(chat.router, prefix="/chat", tags=["chat"])
api_router.include_router(sessions.router, prefix="/sessions", tags=["sessions"])
api_router.include_router(textbook.router, prefix="/textbook", tags=["textbook"])