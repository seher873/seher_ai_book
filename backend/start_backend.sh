#!/bin/bash
# Script to start the Physical AI Textbook RAG Chatbot backend

# Navigate to the backend directory
cd "$(dirname "$0")"

# Activate virtual environment if it exists
if [ -d "venv" ]; then
    source venv/bin/activate
elif [ -d ".venv" ]; then
    source .venv/bin/activate
fi

# Start the backend server
echo "Starting Physical AI Textbook RAG Chatbot backend..."
uvicorn main:app --host 0.0.0.0 --port 8000