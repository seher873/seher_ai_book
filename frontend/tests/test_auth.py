"""
Test suite for the authentication module
Following TDD approach: write tests first, then implement functionality
"""
import pytest
from fastapi.testclient import TestClient
from fastapi import FastAPI
from unittest.mock import patch, MagicMock
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool
from backend.auth.main import init_auth_routes
from backend.auth.database import Base
from backend.auth.schemas import SignupRequest, SigninRequest, UserBackground


# Create an in-memory SQLite database for testing
SQLALCHEMY_DATABASE_URL = "sqlite:///./test.db"

engine = create_engine(
    SQLALCHEMY_DATABASE_URL,
    connect_args={"check_same_thread": False},
    poolclass=StaticPool,
)
TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Create tables
Base.metadata.create_all(bind=engine)


# Override the get_db function for testing
def override_get_db():
    try:
        db = TestingSessionLocal()
        yield db
    finally:
        db.close()


@pytest.fixture
def app():
    """Create a test FastAPI app with auth routes"""
    app = FastAPI()
    init_auth_routes(app)
    
    # Override the database dependency
    from backend.auth.main import get_db
    app.dependency_overrides[get_db] = override_get_db
    
    return app


@pytest.fixture
def client(app):
    """Create a test client"""
    return TestClient(app)


def test_signup_request_model():
    """Test that SignupRequest model works correctly with background fields"""
    software_bg = UserBackground(level="intermediate", languages=["Python", "JavaScript"])
    hardware_bg = UserBackground(experience="beginner", tools=["Arduino", "Raspberry Pi"])
    
    signup_data = SignupRequest(
        email="test@example.com",
        password="securePassword123",
        name="Test User",
        software_background=software_bg,
        hardware_background=hardware_bg
    )
    
    assert signup_data.email == "test@example.com"
    assert signup_data.password == "securePassword123"
    assert signup_data.name == "Test User"
    assert signup_data.software_background.level == "intermediate"
    assert "Python" in signup_data.software_background.languages
    assert signup_data.hardware_background.experience == "beginner"
    assert "Arduino" in signup_data.hardware_background.tools


def test_signup_request_model_optional_background():
    """Test that SignupRequest model works with optional background fields"""
    signup_data = SignupRequest(
        email="test@example.com",
        password="securePassword123",
        name="Test User"
    )
    
    assert signup_data.email == "test@example.com"
    assert signup_data.password == "securePassword123"
    assert signup_data.name == "Test User"
    assert signup_data.software_background is None
    assert signup_data.hardware_background is None


def test_signin_request_model():
    """Test that SigninRequest model works correctly"""
    signin_data = SigninRequest(
        email="test@example.com",
        password="securePassword123"
    )
    
    assert signin_data.email == "test@example.com"
    assert signin_data.password == "securePassword123"


@patch('backend.auth.services.create_user')
@patch('backend.auth.utils.create_access_token')
def test_signup_success(mock_create_token, mock_create_user, client):
    """Test successful signup"""
    # Mock the services
    mock_create_token.return_value = "test_token"
    mock_user = MagicMock()
    mock_user.id = 1
    mock_user.email = "test@example.com"
    mock_user.name = "Test User"
    mock_user.is_verified = False
    mock_user.software_background = None
    mock_user.hardware_background = None
    mock_user.created_at = "2023-01-01T00:00:00"
    mock_create_user.return_value = mock_user
    
    signup_payload = {
        "email": "test@example.com",
        "password": "securePassword123",
        "name": "Test User",
        "software_background": {
            "level": "intermediate",
            "languages": ["Python", "JavaScript"]
        },
        "hardware_background": {
            "experience": "beginner",
            "tools": ["Arduino", "Raspberry Pi"]
        }
    }
    
    response = client.post("/auth/signup", json=signup_payload)
    
    assert response.status_code == 200
    data = response.json()
    assert data["access_token"] == "test_token"
    assert data["token_type"] == "bearer"
    assert data["user"]["email"] == "test@example.com"


@patch('backend.auth.services.create_user')
def test_signup_failure(mock_create_user, client):
    """Test signup failure with invalid data"""
    # Mock the service to raise an exception
    mock_create_user.side_effect = Exception("User already exists")
    
    signup_payload = {
        "email": "test@example.com",
        "password": "short",  # This might cause validation error
        "name": "Test User"
    }
    
    response = client.post("/auth/signup", json=signup_payload)
    
    assert response.status_code == 422  # Validation error


@patch('backend.auth.services.authenticate_user')
@patch('backend.auth.utils.create_access_token')
def test_signin_success(mock_create_token, mock_auth_user, client):
    """Test successful signin"""
    # Mock the services
    mock_create_token.return_value = "test_token"
    mock_user = MagicMock()
    mock_user.id = 1
    mock_user.email = "test@example.com"
    mock_user.name = "Test User"
    mock_user.is_active = True
    mock_user.is_verified = True
    mock_user.software_background = None
    mock_user.hardware_background = None
    mock_user.created_at = "2023-01-01T00:00:00"
    mock_auth_user.return_value = mock_user
    
    signin_payload = {
        "email": "test@example.com",
        "password": "securePassword123"
    }
    
    response = client.post("/auth/signin", json=signin_payload)
    
    assert response.status_code == 200
    data = response.json()
    assert data["access_token"] == "test_token"
    assert data["token_type"] == "bearer"
    assert data["user"]["email"] == "test@example.com"


@patch('backend.auth.services.authenticate_user')
def test_signin_failure(mock_auth_user, client):
    """Test signin failure with invalid credentials"""
    # Mock the service to return None (invalid credentials)
    mock_auth_user.return_value = None
    
    signin_payload = {
        "email": "test@example.com",
        "password": "wrongPassword123"
    }
    
    response = client.post("/auth/signin", json=signin_payload)
    
    assert response.status_code == 401


def test_user_background_model():
    """Test that UserBackground model works correctly"""
    bg = UserBackground(
        level="advanced",
        languages=["Python", "C++", "Rust"],
        experience="expert",
        tools=["Docker", "Kubernetes", "Terraform"]
    )
    
    assert bg.level == "advanced"
    assert "Python" in bg.languages
    assert "Rust" in bg.languages
    assert len(bg.languages) == 3
    assert bg.experience == "expert"
    assert "Docker" in bg.tools
    assert "Terraform" in bg.tools
    assert len(bg.tools) == 3


def test_user_background_model_defaults():
    """Test that UserBackground model has correct defaults"""
    bg = UserBackground()
    
    assert bg.level is None
    assert bg.experience is None
    assert bg.languages == []
    assert bg.tools == []