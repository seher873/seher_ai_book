# RAG Chatbot Backend

This is a backend service for a Retrieval-Augmented Generation (RAG) chatbot that uses Qdrant for vector storage and Cohere for embeddings.

## Features

- FastAPI-based API server
- Vector search using Qdrant
- Text embeddings using Cohere
- RAG (Retrieval-Augmented Generation) functionality

## Requirements

- Python 3.8+
- Dependencies listed in `requirements.txt`

## Environment Variables

Create a `.env` file with the following variables:

```env
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
COHERE_API_KEY=your_cohere_api_key
OPENROUTER_API_KEY=your_openrouter_api_key  # Optional, if using OpenRouter
```

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Set up your environment variables in a `.env` file

3. Run the server:
   ```bash
   uvicorn retry_rag_api_port8002:app --host 0.0.0.0 --port 8002
   ```

## API Endpoints

- `POST /query` - Query the RAG system
  - Request: `{"query": "your question"}`
  - Response: `{"results": ["chunk1 text", "chunk2 text", "chunk3 text"]}`

- `GET /health` - Check server health

## Scripts

- `ingest_docs.py` - Script to ingest documents into Qdrant
- `main.py` - Main application file
- `retry_rag_api_port8002.py` - Production-ready RAG API server