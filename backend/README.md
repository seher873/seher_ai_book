# Physical AI Textbook RAG Chatbot Backend

This is the backend for a RAG (Retrieval-Augmented Generation) chatbot that works with the Physical AI & Humanoid Robotics textbook.

## Deployment on Hugging Face Spaces

This repository is configured to be deployed on Hugging Face Spaces using Docker.

### Environment Variables Required

Your Space will need the following environment variables set in the Secrets section:

- `COHERE_API_KEY`: Your Cohere API key for embeddings
- `OPENROUTER_API_KEY`: Your OpenRouter API key for generation
- `QDRANT_URL`: Your Qdrant database URL
- `QDRANT_API_KEY`: Your Qdrant API key

### How It Works

1. The backend uses FastAPI to serve the RAG chatbot API
2. It connects to Qdrant for vector storage and retrieval
3. It uses Cohere for embeddings and OpenRouter for response generation
4. The API endpoints include:
   - `/` - Health check endpoint
   - `/chat` - Main chat endpoint for RAG queries
   - `/ingest` - Endpoint for ingesting documents
   - `/health` - Health check endpoint

### Architecture

The RAG system works as follows:

1. User Query
2. Embedding Generation with Cohere
3. Vector Search in Qdrant
4. Context Retrieval
5. Response Generation with OpenRouter
6. RAG Response