# Physical AI Textbook Backend

This is the backend for the Physical AI & Humanoid Robotics Textbook with RAG chatbot functionality.

## Tech Stack

- **FastAPI**: Modern Python web framework
- **Qdrant**: Vector database for semantic search
- **Cohere**: Embedding generation for RAG
- **OpenRouter**: LLM integration
- **SQLAlchemy**: Database ORM
- **SQLite**: Development database

## Setup

### Prerequisites

- Python 3.9+
- pip

### Installation

1. Create a virtual environment:

```bash
cd backend
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
# Edit .env with your actual API keys and configuration
```

### Environment Variables

- `COHERE_API_KEY`: Your Cohere API key for embeddings
- `OPENROUTER_API_KEY`: Your OpenRouter API key for LLM access
- `OPENROUTER_MODEL`: The model to use (default: xiaomi/mimo-v2-flash)
- `QDRANT_URL`: Your Qdrant instance URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `JWT_SECRET`: Secret key for JWT token generation
- `DATABASE_URL`: Database connection string (default: SQLite)

## Running the Application

### Development

```bash
# Activate virtual environment
source venv/bin/activate

# Run the application
uvicorn main:app --reload
```

The API will be available at `http://localhost:8000`.

### Production

```bash
# Using uvicorn
uvicorn main:app --host 0.0.0.0 --port 8000

# Or using the main.py directly
python main.py
```

## API Endpoints

### Health Check
- `GET /` - Root endpoint
- `GET /health` - Health check

### RAG Chatbot
- `POST /api/chat` - Query the textbook chatbot
- `POST /api/ingest` - Ingest documents into vector database

### Authentication
- `POST /api/register` - User registration
- `POST /api/login` - User login
- `POST /api/token` - Get access token

## Ingesting Content

To ingest textbook content into the vector database:

```bash
python scripts/ingest_content.py [path_to_docs_directory]
```

By default, it will look for content in `../../frontend/docs`.

## Docker Deployment

To build and run with Docker:

```bash
# Build the image
docker build -t ai-textbook-backend .

# Run the container
docker run -p 8000:8000 --env-file .env ai-textbook-backend
```

## Architecture

### RAG Pipeline
```
User Query → Cohere Embedding → Qdrant Search → Context Retrieval →
OpenRouter LLM → Response
```

## Deployment

The backend is designed to be deployed to Hugging Face Spaces or similar platforms that support FastAPI applications.

For Hugging Face Spaces:
1. Create a Space with Docker or Python environment
2. Add your environment variables as secrets
3. The application will run using the provided Dockerfile or by running `python main.py`