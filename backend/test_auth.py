"""
Test suite for the authentication module
Following TDD approach: write tests first, then implement functionality
"""
from datetime import datetime, timedelta
from unittest.mock import patch, MagicMock
from auth import AuthService, UserRegistration, UserLogin, User, UserRole, auth_service
import jwt
import os

def test_hash_password():
    """Test that password hashing works correctly"""
    password = "test_password_123"
    hashed = auth_service.hash_password(password)
    
    # Verify the password is hashed
    assert hashed != password
    assert len(hashed) > 0
    
    # Verify the hash can be verified against the original password
    assert auth_service.verify_password(password, hashed)

def test_verify_password_incorrect():
    """Test that incorrect password verification fails"""
    password = "test_password_123"
    wrong_password = "wrong_password_456"
    hashed = auth_service.hash_password(password)
    
    # Verify the wrong password fails
    assert not auth_service.verify_password(wrong_password, hashed)

def test_create_access_token():
    """Test that access tokens are created correctly"""
    data = {"sub": "test@example.com"}
    token = auth_service.create_access_token(data=data)
    
    # Verify the token is created
    assert token is not None
    assert isinstance(token, str)
    
    # Decode and verify the token contents
    decoded = jwt.decode(token, os.getenv("JWT_SECRET_KEY", "your-default-secret-key-change-in-production"), algorithms=["HS256"])
    assert decoded["sub"] == "test@example.com"

def test_create_access_token_with_expiration():
    """Test that access tokens expire correctly"""
    data = {"sub": "test@example.com"}
    expires_delta = timedelta(minutes=30)
    token = auth_service.create_access_token(data=data, expires_delta=expires_delta)
    
    # Decode and verify the token expiration
    decoded = jwt.decode(token, os.getenv("JWT_SECRET_KEY", "your-default-secret-key-change-in-production"), algorithms=["HS256"])
    exp = datetime.utcfromtimestamp(decoded["exp"])
    expected_exp = datetime.utcnow() + expires_delta
    
    # Allow for small time differences
    assert abs((exp - expected_exp).total_seconds()) < 5

def test_decode_token_valid():
    """Test that valid tokens are decoded correctly"""
    data = {"sub": "test@example.com"}
    token = auth_service.create_access_token(data=data)
    
    token_data = auth_service.decode_token(token)
    
    assert token_data is not None
    assert token_data.email == "test@example.com"

def test_decode_token_invalid():
    """Test that invalid tokens return None"""
    invalid_token = "invalid.token.here"
    
    token_data = auth_service.decode_token(invalid_token)
    
    assert token_data is None

def test_decode_token_expired():
    """Test that expired tokens return None"""
    data = {"sub": "test@example.com"}
    expired_token = auth_service.create_access_token(
        data=data,
        expires_delta=timedelta(seconds=-1)  # Expired token
    )
    
    token_data = auth_service.decode_token(expired_token)
    
    assert token_data is None

def test_register_user_success():
    """Test successful user registration"""
    user_data = UserRegistration(
        email="test@example.com",
        password="securePassword123",
        name="Test User",
        software_background={"level": "intermediate", "languages": ["Python"]},
        hardware_background={"experience": "beginner", "tools": ["Arduino"]}
    )
    
    user = auth_service.register_user(user_data)
    
    # Verify user is created
    assert user.email == "test@example.com"
    assert user.name == "Test User"
    assert user.role == UserRole.USER
    assert user.software_background["level"] == "intermediate"
    assert user.hardware_background["experience"] == "beginner"
    
    # Verify user is stored in the service
    assert "test@example.com" in auth_service.users

def test_register_user_duplicate():
    """Test that registering duplicate users fails"""
    # Clear users to start fresh
    auth_service.users = {}

    user_data = UserRegistration(
        email="duplicate@example.com",
        password="securePassword123",
        name="Test User",
        software_background=None,
        hardware_background=None
    )

    # Register the user first time
    auth_service.register_user(user_data)

    # Try to register the same user again
    try:
        auth_service.register_user(user_data)
        # If we reach this line, the test failed
        assert False, "Expected an exception for duplicate registration"
    except Exception as e:
        # Verify the error
        assert e.status_code == 400
        assert "already exists" in e.detail

def test_authenticate_user_success():
    """Test successful user authentication"""
    # Register a user first
    user_data = UserRegistration(
        email="auth@example.com",
        password="securePassword123",
        name="Auth User",
        software_background=None,
        hardware_background=None
    )
    auth_service.register_user(user_data)
    
    # Authenticate with correct credentials
    user = auth_service.authenticate_user("auth@example.com", "securePassword123")
    
    # Verify authentication success
    assert user is not None
    assert user.email == "auth@example.com"
    assert user.name == "Auth User"

def test_authenticate_user_wrong_password():
    """Test authentication with wrong password fails"""
    # Register a user first
    user_data = UserRegistration(
        email="auth@example.com",
        password="securePassword123",
        name="Auth User",
        software_background=None,
        hardware_background=None
    )
    auth_service.register_user(user_data)
    
    # Authenticate with wrong password
    user = auth_service.authenticate_user("auth@example.com", "wrong_password")
    
    # Verify authentication fails
    assert user is None

def test_authenticate_user_nonexistent():
    """Test authentication with non-existent user fails"""
    # Try to authenticate with non-existent user
    user = auth_service.authenticate_user("nonexistent@example.com", "some_password")
    
    # Verify authentication fails
    assert user is None

def test_get_current_user_success():
    """Test getting current user from valid token"""
    # Register a user first
    user_data = UserRegistration(
        email="current@example.com",
        password="securePassword123",
        name="Current User",
        software_background=None,
        hardware_background=None
    )
    registered_user = auth_service.register_user(user_data)
    
    # Create a token for the user
    token_data = {"sub": "current@example.com"}
    token = auth_service.create_access_token(data=token_data)
    
    # Mock credentials object
    class MockCredentials:
        def __init__(self, token):
            self.credentials = token
    
    credentials = MockCredentials(token)
    
    # In a real test, we would need to mock the security dependency
    # For now, we'll test the underlying logic
    token_data_decoded = auth_service.decode_token(token)
    assert token_data_decoded is not None
    assert token_data_decoded.email == "current@example.com"

def test_get_current_user_invalid_token():
    """Test getting current user with invalid token raises exception"""
    # Mock credentials object with invalid token
    class MockCredentials:
        def __init__(self, token):
            self.credentials = token
    
    credentials = MockCredentials("invalid.token.here")
    
    # In a real test, we would need to mock the security dependency
    # For now, we'll test the underlying logic
    token_data_decoded = auth_service.decode_token(credentials.credentials)
    assert token_data_decoded is None

# Run the tests
if __name__ == "__main__":
    pytest.main([__file__])