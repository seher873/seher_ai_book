---
title: Physical AI Textbook - Backend
emoji: 🤖
colorFrom: blue
colorTo: yellow
sdk: docker
sdk_version: "3.11"
python_version: "3.11"
app_file: main.py
pinned: false
---

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
- API keys for Cohere, OpenRouter, and Qdrant
- Git (for cloning the repository)

### Installation

1. Clone the repository:
```bash
git clone https://github.com/seher873/seher_ai_book.git
cd seher_ai_book/backend
```

2. Create and activate virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

4. Set up environment variables:
```bash
cp .env.example .env
# Edit .env with your actual API keys and configuration
```

#### Required Environment Variables:
- `COHERE_API_KEY` - Cohere API key for document embeddings (get from https://dashboard.cohere.com/)
- `OPENROUTER_API_KEY` - OpenRouter API key for LLM access (get from https://openrouter.ai/keys)
- `OPENROUTER_MODEL` - Model name (default: google/gemini-flash-1.5 or xiaomi/mimo-v2-flash:free)
- `QDRANT_URL` - Qdrant instance URL (either cloud instance or local)
- `QDRANT_API_KEY` - Qdrant API key (for cloud instances)
- `JWT_SECRET` - Secret key for JWT token encryption (generate a strong random key)
- `DATABASE_URL` - SQLite database URL (default: sqlite:///./users.db)

#### Setting Up Required Services:

**Cohere API Key:**
1. Visit https://dashboard.cohere.com/
2. Sign up for an account
3. Create a new API key in the API Keys section
4. Add it to your .env file as `COHERE_API_KEY`

**OpenRouter API Key:**
1. Visit https://openrouter.ai/
2. Sign up for an account
3. Navigate to Keys section and create a new key
4. Add it to your .env file as `OPENROUTER_API_KEY`

**Qdrant Vector Database:**
Option 1 - Qdrant Cloud (Recommended for production):
1. Visit https://cloud.qdrant.io/
2. Sign up for an account
3. Create a new cluster
4. Get the URL and API key from the cluster dashboard
5. The correct URL format should be: `https://<cluster-id>.gcp.qdrant.tech` (without port in URL, as it's handled automatically)

Option 2 - Local Qdrant (For development):
1. Install Docker Desktop: https://www.docker.com/products/docker-desktop/
2. Enable WSL 2 integration in Docker Desktop settings
3. Run: `docker run -d --name qdrant-local -p 6333:6333 qdrant/qdrant`
4. Use `http://localhost:6333` as your QDRANT_URL
5. No API key needed for local instance
6. To stop the container: `docker stop qdrant-local`
7. To restart the container: `docker start qdrant-local`

Option 3 - Alternative Local Setup (using Docker in Windows, accessed from WSL):
If Docker is not working directly in WSL:
1. Make sure Docker Desktop is running on Windows
2. Enable WSL integration in Docker Desktop settings
3. Access the Qdrant instance from WSL using the Windows host IP

**Docker Integration with WSL 2:**
If you're using WSL 2 and Docker Desktop on Windows:
1. Open Docker Desktop
2. Go to Settings > General
3. Ensure "Use WSL 2 based engine" is checked
4. Go to Settings > Resources > WSL Integration
5. Enable integration for your WSL distribution
6. Restart Docker Desktop after making changes

**JWT Secret:**
Generate a secure JWT secret using:
```bash
python -c "import secrets; print(secrets.token_urlsafe(32))"
```

5. Initialize database:
```bash
python scripts/init_db.py
```

6. (Optional) Ingest content to populate the knowledge base:
```bash
python scripts/ingest_all_content.py
```

## Running the Server

### Prerequisites
Before running the server, make sure:
1. All environment variables are properly set in your `.env` file
2. If using local Qdrant, ensure the Qdrant container is running:
   ```bash
   docker run -d --name qdrant-local -p 6333:6333 qdrant/qdrant
   ```
3. Verify your configuration:
   ```bash
   python check_config.py
   ```

### Development
```bash
python main.py
```
Or with auto-reload:
```bash
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

### Production
```bash
uvicorn main:app --host 0.0.0.0 --port 8000 --workers 4
```

### With Docker
```bash
docker build -t rag-chatbot-backend .
docker run -p 8000:8000 --env-file .env rag-chatbot-backend
```

### Configuration Verification
After starting the server, verify that all services are properly connected by visiting:
- Health check: http://localhost:8000/health
- API documentation: http://localhost:8000/docs
- Root endpoint: http://localhost:8000/

The health endpoint should return status "healthy" with all services showing as "up".

### Starting Qdrant Locally (if using local Qdrant)
If you're using local Qdrant for development, you need to start the Qdrant container before running the backend:

```bash
# Start Qdrant container in the background
docker run -d --name qdrant-local -p 6333:6333 qdrant/qdrant

# Verify Qdrant is running
docker ps

# To stop Qdrant when done
docker stop qdrant-local

# To start Qdrant again later
docker start qdrant-local
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

### Configuration Verification Steps

Before running the server, ensure all configurations are correct:

1. **Verify Environment Variables:**
   ```bash
   # Check if environment variables are properly set
   python -c "from dotenv import load_dotenv; import os; load_dotenv(); print('COHERE:', bool(os.getenv('COHERE_API_KEY'))); print('OPENROUTER:', bool(os.getenv('OPENROUTER_API_KEY'))); print('QDRANT_URL:', bool(os.getenv('QDRANT_URL')))"
   ```

2. **Test Service Connections:**
   ```bash
   # Run the test scripts to verify connections
   python test_openrouter.py
   python test_backend_api.py
   ```

3. **Comprehensive Configuration Check:**
   ```bash
   # Run the configuration verifier to check all services
   python check_config.py
   ```

3. **Check Qdrant Connection:**
   ```bash
   # Test if Qdrant is accessible
   python -c "import qdrant_client; client=qdrant_client.QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY')); print(client.get_collections())"
   ```

### Common Issues

1. **Services not initialized**
   - Check `.env` file has all required API keys
   - Verify API keys are valid and not expired
   - Confirm network connectivity to external services
   - Test individual service connectivity with test scripts

2. **Database errors**
   - Run `python scripts/init_db.py` to reinitialize
   - Check database file permissions
   - Verify `DATABASE_URL` is correctly formatted

3. **Qdrant connection errors**
   - Verify `QDRANT_URL` and `QDRANT_API_KEY`
   - Ensure collection exists
   - Check firewall/network settings if using cloud Qdrant
   - For Qdrant Cloud, make sure the URL format is correct: `https://<cluster-id>.gcp.cloud.qdrant.io:6333` (includes port 6333)
   - For local Qdrant, ensure the service is running: `docker ps | grep qdrant`
   - Test Qdrant connectivity separately: `curl -X GET "https://<your-cluster>.gcp.cloud.qdrant.io:6333/collections" -H "api-key: <your-api-key>"`

4. **Import errors**
   - Ensure virtual environment is activated
   - Reinstall dependencies: `pip install -r requirements.txt`
   - Check Python version compatibility (requires 3.9+)

5. **Port already in use**
   - Check if port 8000 is already in use: `lsof -i :8000`
   - Kill any existing processes: `lsof -ti:8000 | xargs kill -9` (use carefully!)
   - Use a different port: `uvicorn main:app --host 0.0.0.0 --port 8001`

6. **Rate Limiting Issues**
   - If seeing rate limit errors, verify your API key quotas with respective services
   - Check `RATE_LIMITING_SUMMARY.md` for current limits

7. **SSL/Certificate Issues**
   - If encountering SSL errors with external APIs, try setting: `export CURL_CA_BUNDLE=""` or `export REQUESTS_CA_BUNDLE=""`

8. **Common Configuration Issues**
   - If getting 400 error with Cohere, ensure you're providing the correct input_type parameter
   - If getting 404 error with OpenRouter, verify the model name exists on OpenRouter
   - If getting 404 error with Qdrant Cloud, ensure the URL format is correct: `https://<cluster-id>.gcp.qdrant.tech`
   - If getting connection refused error with local Qdrant, ensure Docker is installed and the Qdrant container is running
   - If Docker isn't working in WSL, ensure WSL integration is enabled in Docker Desktop settings
   - Check that all API keys are valid and not expired

## Deployment on Hugging Face Spaces

This backend can be deployed on Hugging Face Spaces using Docker.

### Required Secrets
Set these in your Space's Settings > Variables and secrets:
- `COHERE_API_KEY` - Cohere API key
- `OPENROUTER_API_KEY` - OpenRouter API key
- `QDRANT_URL` - Qdrant instance URL
- `QDRANT_API_KEY` - Qdrant API key
- `JWT_SECRET` - JWT secret key
- `DATABASE_URL` - Database URL (optional, defaults to SQLite)

### Space Configuration
1. Create a new Space on Hugging Face
2. Select "Docker" SDK
3. Choose "GPU" or "CPU" hardware (CPU is sufficient for most use cases)
4. Set the secrets in Space settings
5. Add the following to your `Dockerfile` if not already present:
   ```Dockerfile
   FROM python:3.11-slim

   WORKDIR /app

   COPY requirements.txt .
   RUN pip install -r requirements.txt

   COPY . .

   EXPOSE 8000

   CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
   ```

### Runtime Configuration
For Hugging Face Spaces, ensure your `app.py` or main file includes proper startup and shutdown events for service initialization as implemented in the current codebase.

## Contributing

1. Follow the project structure
2. Add tests for new features
3. Update documentation
4. Follow Python best practices

