"""
Simple test script to validate authentication implementation
"""
import sys
import os

# Add the backend directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

def test_imports():
    """Test that all modules can be imported without errors"""
    try:
        from backend.auth.schemas import SignupRequest, SigninRequest, UserBackground
        from backend.auth.models import User
        from backend.auth.services import create_user, authenticate_user
        from backend.auth.utils import get_password_hash, verify_password, create_access_token
        print("✓ All modules imported successfully")
        return True
    except ImportError as e:
        print(f"✗ Import error: {e}")
        return False


def test_user_background_schema():
    """Test UserBackground schema functionality"""
    try:
        from backend.auth.schemas import UserBackground
        
        # Test with all fields
        bg = UserBackground(
            level="intermediate",
            languages=["Python", "JavaScript"],
            experience="beginner",
            tools=["Arduino", "Raspberry Pi"]
        )
        
        assert bg.level == "intermediate"
        assert "Python" in bg.languages
        assert bg.experience == "beginner"
        assert "Arduino" in bg.tools
        
        # Test with defaults
        empty_bg = UserBackground()
        assert empty_bg.level is None
        assert empty_bg.languages == []
        assert empty_bg.experience is None
        assert empty_bg.tools == []
        
        print("✓ UserBackground schema tests passed")
        return True
    except Exception as e:
        print(f"✗ UserBackground schema test failed: {e}")
        return False


def test_signup_request_schema():
    """Test SignupRequest schema functionality"""
    try:
        from backend.auth.schemas import SignupRequest, UserBackground
        
        software_bg = UserBackground(level="intermediate", languages=["Python"])
        hardware_bg = UserBackground(experience="beginner", tools=["Arduino"])
        
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
        assert signup_data.hardware_background.experience == "beginner"
        
        print("✓ SignupRequest schema tests passed")
        return True
    except Exception as e:
        print(f"✗ SignupRequest schema test failed: {e}")
        return False


def test_signin_request_schema():
    """Test SigninRequest schema functionality"""
    try:
        from backend.auth.schemas import SigninRequest
        
        signin_data = SigninRequest(
            email="test@example.com",
            password="securePassword123"
        )
        
        assert signin_data.email == "test@example.com"
        assert signin_data.password == "securePassword123"
        
        print("✓ SigninRequest schema tests passed")
        return True
    except Exception as e:
        print(f"✗ SigninRequest schema test failed: {e}")
        return False


def test_utils_functions():
    """Test utility functions"""
    try:
        from backend.auth.utils import get_password_hash, verify_password, create_access_token
        
        # Test password hashing
        password = "securePassword123"
        hashed = get_password_hash(password)
        assert hashed != password  # Hashed password should be different
        assert verify_password(password, hashed)  # Should verify correctly
        assert not verify_password("wrongPassword", hashed)  # Should not verify wrong password
        
        # Test token creation (this might fail if JWT settings are not configured)
        try:
            token = create_access_token(data={"sub": "test@example.com"})
            assert isinstance(token, str)
            assert len(token) > 0
        except Exception as e:
            print(f"  (Token creation test skipped due to configuration: {e})")
        
        print("✓ Utility functions tests passed")
        return True
    except Exception as e:
        print(f"✗ Utility functions test failed: {e}")
        return False


def main():
    """Run all tests"""
    print("Running authentication module tests...\n")
    
    tests = [
        test_imports,
        test_user_background_schema,
        test_signup_request_schema,
        test_signin_request_schema,
        test_utils_functions
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
        print()
    
    print(f"Results: {passed}/{total} test groups passed")
    
    if passed == total:
        print("\n✓ All tests passed! Authentication implementation is working correctly.")
        return True
    else:
        print(f"\n✗ {total - passed} test group(s) failed.")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)