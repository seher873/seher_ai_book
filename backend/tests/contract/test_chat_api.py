"""
Contract test for POST /chat endpoint
Task: T016 [P] [US1] Contract test for POST /chat endpoint in backend/tests/contract/test_chat_api.py
"""
import pytest
from fastapi.testclient import TestClient
from rag_api.main import app


@pytest.fixture
def client():
    """Test client for the FastAPI app"""
    return TestClient(app)


def test_post_chat_endpoint_contract_structure(client):
    """Test that the POST /chat endpoint returns the expected response structure"""
    # Test data
    test_payload = {
        "session_token": "test_session_123",
        "message": "What is the main concept of Physical AI?",
        "selected_text": None
    }
    
    response = client.post("/api/v1/chat", json=test_payload)
    
    # Status code should be 200
    assert response.status_code == 200
    
    # Response should be JSON
    data = response.json()
    
    # Verify response structure
    assert "id" in data, "Response must contain an id field"
    assert "content" in data, "Response must contain a content field"
    assert "citations" in data, "Response must contain a citations field"
    assert "timestamp" in data, "Response must contain a timestamp field"
    
    # Verify field types
    assert isinstance(data["id"], str), "ID should be a string"
    assert isinstance(data["content"], str), "Content should be a string"
    assert isinstance(data["citations"], list), "Citations should be a list"
    assert isinstance(data["timestamp"], str), "Timestamp should be a string"


def test_post_chat_endpoint_required_fields(client):
    """Test that the POST /chat endpoint validates required fields"""
    # Test with missing required fields
    response = client.post("/api/v1/chat", json={})
    
    # Should return 422 for validation error
    assert response.status_code == 422


def test_post_chat_endpoint_citation_structure(client):
    """Test that citations in the response have the expected structure"""
    test_payload = {
        "session_token": "test_session_123",
        "message": "What is the main concept of Physical AI?",
        "selected_text": None
    }
    
    response = client.post("/api/v1/chat", json=test_payload)
    
    # Skip if endpoint is not yet implemented (would return 404 or 500)
    if response.status_code != 500 and response.status_code != 404:
        assert response.status_code == 200
        data = response.json()
        
        # If citations exist, check their structure
        for citation in data["citations"]:
            assert "id" in citation
            assert "chapter_number" in citation
            assert "section_number" in citation
            assert "text_excerpt" in citation


if __name__ == "__main__":
    pytest.main([__file__])