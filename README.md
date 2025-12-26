# Physical AI Textbook RAG Chatbot

This is a comprehensive Book + Chatbot (RAG) system for the Physical AI & Humanoid Robotics textbook with authentication.

## Project Structure

```
my_ai_book/
├── backend/
│   ├── auth/                 # Authentication module
│   │   ├── __init__.py
│   │   ├── main.py          # Auth routes and initialization
│   │   ├── models.py        # Database models
│   │   ├── schemas.py       # Pydantic schemas
│   │   ├── services.py      # Business logic
│   │   ├── utils.py         # Utility functions
│   │   ├── config.py        # Configuration
│   │   ├── database.py      # Database setup
│   │   └── dependencies.py  # FastAPI dependencies
│   ├── main.py              # Main FastAPI application
│   ├── models.py            # Database models
│   ├── schemas.py           # Pydantic schemas
│   ├── database.py          # Database setup
│   └── utils.py             # Utility functions
├── src/                     # Frontend source
│   ├── components/
│   │   └── Auth/            # Authentication components
│   ├── pages/
│   │   ├── auth/
│   │   └── dashboard/
│   ├── utils/
│   └── theme/
├── specs/                   # Specifications and plans
│   └── 008-authentication-improvement/
│       ├── spec.md
│       ├── plan.md
│       └── tasks.md
├── tests/                   # Test files
├── requirements.txt         # Backend dependencies
├── package.json             # Frontend dependencies
└── README.md
```

## Architecture

### Backend
- **FastAPI**: Main web framework
- **Qdrant**: Vector database for RAG
- **Cohere**: Embedding generation
- **OpenRouter**: LLM integration
- **SQLAlchemy**: Database ORM
- **better-auth**: Authentication system

### Frontend
- **React**: UI framework
- **Docusaurus**: Documentation
- **Tailwind CSS**: Styling

## Setup

### Backend
```bash
cd backend
pip install -r requirements.txt
uvicorn main:app --reload
```

### Frontend
```bash
cd src
npm install
npm start
```

## Features

1. **RAG Chatbot**: Retrieve and generate responses based on the textbook content
2. **Authentication**: Secure signup and signin with user background information
3. **Vector Search**: Fast semantic search using Qdrant
4. **API Endpoints**: Well-defined REST API for all functionality

## Authentication Flow

1. User registers with email, password, name and background information
2. Credentials are securely hashed and stored
3. JWT token is generated for authenticated sessions
4. Protected endpoints require valid authentication tokens