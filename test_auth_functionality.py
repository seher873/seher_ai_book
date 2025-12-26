"""
Test script to verify authentication functionality
"""
import sys
import os
from datetime import timedelta

# Add the backend directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

def test_password_hashing():
    """Test password hashing functionality"""
    try:
        from backend.auth.utils import get_password_hash, verify_password

        password = "securePass123"  # Shorter password to avoid bcrypt 72-byte limit
        hashed = get_password_hash(password)

        # Verify the password
        assert verify_password(password, hashed), "Password verification failed"
        assert not verify_password("wrongPass123", hashed), "Wrong password should not verify"

        print("✓ Password hashing and verification working correctly")
        return True
    except Exception as e:
        print(f"✗ Password hashing test failed: {e}")
        return False


def test_jwt_token_creation():
    """Test JWT token creation functionality"""
    try:
        from backend.auth.utils import create_access_token
        from backend.auth.config import settings
        
        # Create a token
        data = {"sub": "test@example.com", "exp": 1234567890}
        token = create_access_token(data={"sub": "test@example.com"})
        
        assert isinstance(token, str), "Token should be a string"
        assert len(token) > 0, "Token should not be empty"
        
        print("✓ JWT token creation working correctly")
        return True
    except Exception as e:
        print(f"✗ JWT token creation test failed: {e}")
        return False


def test_user_creation():
    """Test user creation functionality"""
    try:
        from backend.auth.schemas import UserCreate
        from backend.auth.models import User
        from backend.auth.utils import get_password_hash

        # Create a user object
        user_data = UserCreate(
            email="test@example.com",
            password="securePass123",  # Shorter password to avoid bcrypt 72-byte limit
            name="Test User",
            software_background=None,
            hardware_background=None
        )

        # Create a model instance
        hashed_password = get_password_hash(user_data.password)
        user = User(
            email=user_data.email,
            name=user_data.name,
            hashed_password=hashed_password
        )

        assert user.email == "test@example.com"
        assert user.name == "Test User"
        assert user.hashed_password != user_data.password  # Should be hashed

        print("✓ User creation working correctly")
        return True
    except Exception as e:
        print(f"✗ User creation test failed: {e}")
        return False


def test_authentication_flow():
    """Test the complete authentication flow"""
    try:
        from backend.auth.services import create_user, authenticate_user
        from backend.auth.schemas import UserCreate
        from backend.auth.database import SessionLocal, engine
        from backend.auth.models import Base
        from sqlalchemy.orm import Session

        # Create all tables in the database
        Base.metadata.create_all(bind=engine)

        # Create a test database session
        db: Session = SessionLocal()

        try:
            # Create a test user
            user_data = UserCreate(
                email="test@example.com",
                password="securePass123",  # Shorter password to avoid bcrypt 72-byte limit
                name="Test User",
                software_background=None,
                hardware_background=None
            )

            # Create the user in the database
            created_user = create_user(db, user_data)

            # Try to authenticate the user
            authenticated_user = authenticate_user(db, "test@example.com", "securePass123")

            assert authenticated_user is not None, "User should be authenticated"
            assert authenticated_user.email == "test@example.com"

            # Try to authenticate with wrong password
            wrong_auth = authenticate_user(db, "test@example.com", "wrongPass123")
            assert wrong_auth is None, "Should not authenticate with wrong password"

            print("✓ Authentication flow working correctly")
            return True
        finally:
            # Clean up: delete the test user
            if 'created_user' in locals():
                db.delete(created_user)
                db.commit()
            db.close()
    except Exception as e:
        print(f"✗ Authentication flow test failed: {e}")
        return False


def main():
    """Run all authentication functionality tests"""
    print("Testing authentication functionality...\n")
    
    tests = [
        ("Password hashing", test_password_hashing),
        ("JWT token creation", test_jwt_token_creation),
        ("User creation", test_user_creation),
        ("Authentication flow", test_authentication_flow)
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"Running {test_name} test...")
        if test_func():
            passed += 1
        print()
    
    print(f"Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("\n✓ All authentication functionality is working correctly!")
        return True
    else:
        print(f"\n✗ {total - passed} test(s) failed.")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)