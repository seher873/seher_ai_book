"""Main application entry point for the RAG API."""
from fastapi import FastAPI

app = FastAPI(title="RAG Chatbot API", version="1.0.0")


@app.get("/")
async def root():
    return {"message": "RAG Chatbot API is running!"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)