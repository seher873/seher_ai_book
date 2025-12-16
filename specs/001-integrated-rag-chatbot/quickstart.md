# Quickstart Guide: Integrated Read-Only RAG Chatbot

**Date**: 2025-12-16  
**Target Audience**: Developers implementing the RAG chatbot feature  
**Related Documents**: [Implementation Plan](plan.md), [Data Model](data-model.md), [API Contracts](contracts/api-contract.yaml)

## Overview

This quickstart guide provides developers with the essential information needed to begin implementing the RAG chatbot for the Physical AI textbook. It covers the key technologies, setup steps, and development workflow.

## Prerequisites

### System Requirements
- Python 3.11+ with pip
- Node.js 20+ with npm/yarn
- Docker (for local database and vector store)
- Git for version control

### Development Tools
- A code editor with Python and JavaScript/TypeScript support
- OpenAPI/Swagger editor (for API contract validation)
- Postman or similar API testing tool

### API Keys and Credentials
- OpenAI API key (for embeddings and assistant API)
- Neon Postgres connection string (for session storage)
- Qdrant API key and endpoint (for vector storage)

## Project Setup

### 1. Clone and Initialize Repository
```bash
git clone [repository-url]
cd [repository-name]
git checkout 001-integrated-rag-chatbot
```

### 2. Backend Setup
```bash
# Navigate to backend directory
mkdir -p backend/rag_api

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install fastapi uvicorn python-multipart openai psycopg[binary,pool] qdrant-client python-dotenv
pip install pytest pytest-asyncio httpx  # For testing
```

### 3. Frontend Setup
```bash
# The frontend will be integrated into the existing Docusaurus project
# Dependencies will be added to the existing package.json
npm install openai react-markdown  # Add to existing project dependencies
```

### 4. Environment Configuration
Create a `.env` file in the backend directory:
```env
OPENAI_API_KEY=your_openai_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_HOST=your_qdrant_endpoint
DATABASE_URL=your_neon_postgres_connection_string
SECRET_KEY=your_secret_key_for_session_tokens
```

## Development Workflow

### 1. Run Backend Locally
```bash
cd backend
uvicorn rag_api.main:app --reload --port 8000
```

Backend will be available at `http://localhost:8000`

### 2. Run with Docker (Optional)
```bash
docker-compose up -d
```

### 3. Run Tests
```bash
# Backend tests
cd backend
python -m pytest tests/ -v

# Frontend tests
cd frontend
npm test
```

## Key Implementation Areas

### 1. Backend Services
The backend structure follows this organization:

```
backend/rag_api/
├── __init__.py
├── models/
│   ├── session.py      # ChatSession model
│   ├── query.py        # Query model  
│   └── response.py     # Response model
├── services/
│   ├── rag_service.py      # Core RAG logic
│   ├── embedding_service.py # Embedding generation and search
│   └── content_filter.py   # Content moderation
├── api/
│   └── v1/
│       ├── __init__.py
│       ├── chat.py         # Chat endpoints
│       └── sessions.py     # Session endpoints
├── db/
│   ├── connection.py       # Database connection
│   └── migrations.py       # Schema migrations
├── utils/
│   ├── validators.py       # Input validation
│   └── helpers.py          # Utility functions
└── main.py                 # Application entry point
```

### 2. Frontend Integration
The frontend component should be implemented as a React component that can be embedded in the Docusaurus pages:

```
frontend/src/
├── components/
│   └── RagChatbot.jsx    # Main chatbot UI component
├── hooks/
│   └── useRagChat.js     # Custom hook for chat logic
└── services/
    └── ragApi.js         # API client for backend communication
```

### 3. Key Implementation Steps

#### A. Set up the RAG Service
1. Implement vector database indexing of textbook content
2. Create embedding generation for queries
3. Implement semantic search functionality
4. Connect to OpenAI Assistant API for response generation

#### B. Implement Session Management
1. Create secure, temporary session tokens
2. Implement automatic session expiration
3. Store session data in Neon Postgres
4. Add privacy-safe cleanup mechanisms

#### C. Add Content Filtering
1. Integrate OpenAI's Moderation API
2. Filter both queries and responses
3. Implement appropriate user messaging when content is flagged

#### D. Ensure Compliance with Requirements
1. Implement mandatory citation system
2. Ensure book-first principle is enforced
3. Prevent hallucinations through grounding in textbook content
4. Add proper error handling and graceful failure mechanisms

## Testing Strategy

### Unit Tests
- Test individual service functions in isolation
- Validate data model constraints and validation
- Test utility functions

### Integration Tests
- Test API endpoints with mocked external services
- Validate database operations
- Test RAG pipeline components integration

### Contract Tests
- Verify API responses match contract specifications
- Test error response formats
- Validate data model serialization/deserialization

## Deployment

### Backend Deployment Options
1. Deploy to cloud platform (e.g., Railway, Heroku, AWS)
2. Container deployment with Docker
3. Serverless deployment (if supported by requirements)

### Frontend Integration
- The component will be embedded in Docusaurus pages
- No separate deployment needed for frontend

## Troubleshooting

### Common Issues
1. **API Timeout Errors**: Check network connectivity and API key validity
2. **No Citations in Response**: Verify vector database has been properly populated with textbook content
3. **Slow Response Times**: Consider caching frequently requested content
4. **Session Expiration**: Verify session management and cleanup processes

### Debugging Tips
1. Enable detailed logging in development
2. Monitor API token usage
3. Check vector database connection and performance
4. Validate textbook content indexing process

## Resources

- [Feature Specification](spec.md) - Complete feature requirements
- [Implementation Plan](plan.md) - Technical architecture
- [Data Model](data-model.md) - Data structures and relationships
- [API Contracts](contracts/api-contract.yaml) - API specifications
- [OpenAI API Documentation](https://platform.openai.com/docs/) - For assistant and embedding APIs
- [Qdrant Documentation](https://qdrant.tech/documentation/) - For vector database operations