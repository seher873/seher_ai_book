# Implementation Guide: Integrated Read-Only RAG Chatbot

This document provides implementation details for the RAG (Retrieval-Augmented Generation) chatbot feature for the Physical AI textbook.

## Architecture

### Backend Services

The backend consists of several core services:

- **RAG Service**: Handles the retrieval-augmented generation pipeline, including context retrieval, query processing, and response generation with citations.
- **Embedding Service**: Generates vector embeddings for text content and semantic search.
- **Content Filter**: Implements content moderation to prevent harmful queries or responses.
- **Session Management**: Manages user sessions and conversation history.

### Frontend Components

- **RagChatbot Component**: Main chat interface component
- **useRagChat Hook**: Manages chat state and API communication
- **Citation Navigation**: Clickable citations that link to textbook sections

## Key Features Implemented

### User Story 1: Chat with AI Assistant
- Supports both global (full-book corpus) and focused (user-selected text) queries
- Provides responses grounded in textbook content with mandatory citations
- Implements content filtering

### User Story 2: Navigate to Referenced Content
- Citations in chat responses are clickable links
- Links navigate directly to referenced textbook chapters/sections
- Uses structured citation metadata with chapter/section references

### User Story 3: Maintain Conversation Context
- Session-based conversation history
- Context-aware responses that reference previous exchanges
- Limited history to prevent excessive token usage

## API Endpoints

### Chat API
- `POST /api/v1/chat` - Process user queries and get AI responses
- Request body includes session token, message, query type, and optional selected text
- Response includes content, citations, and timestamp

### Session API
- `POST /api/v1/sessions` - Create new chat sessions
- Returns session token for maintaining conversation context

### Textbook API
- `GET /textbook/chapters` - List all textbook chapters
- `GET /textbook/chapters/{chapter_number}` - Get chapter details with sections
- `GET /textbook/chapters/{chapter_number}/sections/{section_number}` - Get specific section content

## Models

### ChatSession
- Manages conversation history and session metadata
- Automatically expires after 24 hours
- Stores conversation context for multi-turn conversations

### Query
- Represents a single query from the user
- Includes conversation context for context-aware processing

### Response
- Contains AI-generated response with citations
- Linked to original query and session

### Citation
- References specific textbook content
- Includes URL for navigation to referenced section

## Configuration

Required environment variables:
- `OPENAI_API_KEY`: OpenAI API key
- `QDRANT_API_KEY`: Qdrant vector database API key
- `QDRANT_HOST`: Qdrant vector database host
- `DATABASE_URL`: PostgreSQL database connection string
- `SECRET_KEY`: Secret key for session tokens