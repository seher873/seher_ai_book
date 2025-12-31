"""
Test script to validate the complete AI-powered book with RAG chatbot solution
"""
import os
import sys
import subprocess
import requests
import time
from pathlib import Path

def test_backend_running():
    """Test if the backend is running and accessible"""
    try:
        response = requests.get("http://localhost:8000/health", timeout=5)
        return response.status_code == 200
    except requests.exceptions.RequestException:
        return False

def test_api_endpoints():
    """Test the main API endpoints"""
    try:
        # Test health endpoint
        health_response = requests.get("http://localhost:8000/health", timeout=5)
        if health_response.status_code != 200:
            return False, "Health endpoint failed"
        
        # Test root endpoint
        root_response = requests.get("http://localhost:8000/", timeout=5)
        if root_response.status_code != 200:
            return False, "Root endpoint failed"
        
        return True, "All API endpoints working"
    except requests.exceptions.RequestException as e:
        return False, f"API test failed: {str(e)}"

def test_chat_functionality():
    """Test the chat functionality (if backend is running)"""
    try:
        # This is a basic test - in a real scenario, we'd need to set up proper authentication
        test_payload = {
            "query": "What is Physical AI?",
            "max_results": 3
        }
        
        response = requests.post(
            "http://localhost:8000/api/chat",
            json=test_payload,
            timeout=15
        )
        
        # We expect either a 401 (unauthorized) or 200 (success) if backend is working
        if response.status_code in [200, 401]:
            return True, f"Chat endpoint accessible (status: {response.status_code})"
        else:
            return False, f"Chat endpoint returned unexpected status: {response.status_code}"
    except requests.exceptions.RequestException as e:
        return False, f"Chat functionality test failed: {str(e)}"

def test_frontend_build():
    """Test if the frontend can be built successfully"""
    frontend_path = Path("frontend")
    
    if not frontend_path.exists():
        return False, "Frontend directory not found"
    
    try:
        # Change to frontend directory
        original_cwd = os.getcwd()
        os.chdir(frontend_path)
        
        # Try to build the frontend
        result = subprocess.run(
            ["npm", "run", "build"],
            capture_output=True,
            text=True,
            timeout=120  # 2 minute timeout
        )
        
        os.chdir(original_cwd)  # Return to original directory
        
        if result.returncode == 0:
            return True, "Frontend build successful"
        else:
            return False, f"Frontend build failed: {result.stderr}"
    except subprocess.TimeoutExpired:
        os.chdir(original_cwd)
        return False, "Frontend build timed out"
    except Exception as e:
        os.chdir(original_cwd)
        return False, f"Frontend build test error: {str(e)}"

def test_ingestion_script():
    """Test if the ingestion script can run (without actually ingesting)"""
    try:
        # Check if the script exists
        script_path = Path("backend/scripts/ingest_content.py")
        if not script_path.exists():
            return False, "Ingestion script not found"
        
        # Check if required dependencies are available
        import markdown
        import bs4
        import cohere
        import qdrant_client
        
        return True, "Ingestion script dependencies available"
    except ImportError as e:
        return False, f"Missing dependency for ingestion: {str(e)}"
    except Exception as e:
        return False, f"Ingestion script test error: {str(e)}"

def main():
    print("Starting validation of the AI-powered book with RAG chatbot solution...")
    print("=" * 70)
    
    all_tests = [
        ("Backend Running", test_backend_running),
        ("API Endpoints", lambda: test_api_endpoints()),
        ("Chat Functionality", lambda: test_chat_functionality()),
        ("Frontend Build", lambda: test_frontend_build()),
        ("Ingestion Script", lambda: test_ingestion_script()),
    ]
    
    results = []
    for test_name, test_func in all_tests:
        print(f"Running {test_name} test...")
        try:
            result, message = test_func()
            results.append((test_name, result, message))
            status = "PASS" if result else "FAIL"
            print(f"  {status}: {message}")
        except Exception as e:
            results.append((test_name, False, f"Test error: {str(e)}"))
            print(f"  ERROR: {str(e)}")
        print()
    
    print("=" * 70)
    print("Test Summary:")
    passed = 0
    for test_name, result, message in results:
        status = "PASS" if result else "FAIL"
        print(f"  {test_name}: {status}")
        if result:
            passed += 1
    
    print(f"\nTotal: {passed}/{len(results)} tests passed")
    
    if passed == len(results):
        print("\n🎉 All tests passed! The solution is working correctly.")
        return True
    else:
        print(f"\n⚠️  {len(results) - passed} test(s) failed. Please check the implementation.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)