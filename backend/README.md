# Physical AI Textbook - Backend

FastAPI-based backend for RAG chatbot and authentication system.

## Project Structure

```
backend/
├── auth/                    # Authentication module
│   ├── __init__.py
│   ├── config.py           # Auth configuration
│   ├── database.py         # Database connection
│   ├── dependencies.py     # FastAPI dependencies
│   ├── main.py            # Auth routes initialization
│   ├── models.py          # SQLAlchemy models
│   ├── schemas.py         # Pydantic schemas
│   ├── services.py        # Business logic
│   └── utils.py           # JWT & password utilities
│
├── scripts/               # Utility scripts
│   ├── __init__.py
│   ├── check_qdrant_data.py    # Check Qdrant collection
│   ├── ingest_all_content.py  # Ingest documents to Qdrant
│   └── init_db.py              # Initialize database
│
├── routes/                # API routes (for future organization)
│   └── __init__.py
│
├── services/              # Business logic services (for future use)
│   └── __init__.py
│
├── utils/                 # Shared utilities (for future use)
│   └── __init__.py
│
├── .env                   # Environment variables (not in git)
├── .env.example          # Example environment file
├── .gitignore            # Git ignore rules
├── Dockerfile            # Docker configuration
├── main.py               # Main FastAPI application
├── pyproject.toml        # Poetry configuration
├── requirements.txt      # Python dependencies
└── README.md             # This file
```

## Features

### RAG Chatbot
- Vector search using Qdrant
- Document embeddings with Cohere
- Response generation with OpenRouter (Gemini 1.5 Flash)
- Context-aware responses

### Authentication System
- User signup/signin
- JWT token generation
- Password hashing with bcrypt
- SQLAlchemy ORM with SQLite
- User profiles with background info

## Setup

### Prerequisites
- Python 3.9+
- Virtual environment (recommended)

### Installation

1. Create and activate virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Set up environment variables:
```bash
cp .env.example .env
# Edit .env with your API keys
```

Required environment variables:
- `COHERE_API_KEY` - Cohere API key for embeddings
- `OPENROUTER_API_KEY` - OpenRouter API key
- `OPENROUTER_MODEL` - Model name (default: xiaomi/mimo-v2-flash)
- `QDRANT_URL` - Qdrant instance URL
- `QDRANT_API_KEY` - Qdrant API key
- `JWT_SECRET` - Secret key for JWT tokens
- `DATABASE_URL` - SQLite database URL

4. Initialize database:
```bash
python scripts/init_db.py
```

5. (Optional) Ingest content:
```bash
python scripts/ingest_all_content.py
```

## Running the Server

### Development
```bash
python main.py
```

### Production
```bash
uvicorn main:app --host 0.0.0.0 --port 8000
```

### With Docker
```bash
docker build -t rag-chatbot-backend .
docker run -p 8000:8000 --env-file .env rag-chatbot-backend
```

## API Endpoints

### Health Check
- `GET /` - Root endpoint
- `GET /health` - Health check

### RAG Chatbot
- `POST /chat` - Query the RAG system
  ```json
  {
    "query": "What is ROS2?",
    "max_results": 5
  }
  ```

- `POST /ingest` - Ingest documents (admin only)
  ```json
  [
    {
      "id": "doc1",
      "content": "Document content...",
      "title": "Chapter 1",
      "path": "/docs/ch1"
    }
  ]
  ```

### Authentication
- `POST /auth/signup` - User registration
  ```json
  {
    "email": "user@example.com",
    "password": "password123",
    "full_name": "John Doe",
    "software_background": "Python, JavaScript",
    "hardware_background": "Arduino, Raspberry Pi"
  }
  ```

- `POST /auth/signin` - User login
  ```json
  {
    "email": "user@example.com",
    "password": "password123"
  }
  ```

## API Documentation

Once the server is running, visit:
- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

## Development

### Code Style
Follow PEP 8 guidelines for Python code.

### Testing
```bash
# Run tests (when available)
pytest
```

### Database Migrations
Currently using SQLite with SQLAlchemy. For production, consider PostgreSQL.

## Architecture

### RAG Pipeline
```
User Query
    ↓
[Embedding Generation - Cohere]
    ↓
[Vector Search - Qdrant]
    ↓
[Context Retrieval]
    ↓
[Response Generation - OpenRouter]
    ↓
Response to User
```

### Authentication Flow
```
User Credentials
    ↓
[Password Verification]
    ↓
[JWT Token Generation]
    ↓
[Token Storage - Client Side]
    ↓
[Protected Routes Access]
```

## Troubleshooting

### Common Issues

1. **Services not initialized**
   - Check `.env` file has all required API keys
   - Verify API keys are valid

2. **Database errors**
   - Run `python scripts/init_db.py` to reinitialize
   - Check database file permissions

3. **Qdrant connection errors**
   - Verify `QDRANT_URL` and `QDRANT_API_KEY`
   - Ensure collection exists

4. **Import errors**
   - Ensure virtual environment is activated
   - Reinstall dependencies: `pip install -r requirements.txt`

## Deployment on Hugging Face Spaces

This backend can be deployed on Hugging Face Spaces using Docker.

### Required Secrets
Set these in your Space's Settings > Variables and secrets:
- `COHERE_API_KEY`
- `OPENROUTER_API_KEY`
- `QDRANT_URL`
- `QDRANT_API_KEY`
- `JWT_SECRET`

## Contributing

1. Follow the project structure
2. Add tests for new features
3. Update documentation
4. Follow Python best practices
