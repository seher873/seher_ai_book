import os
import requests
from urllib.parse import urlparse

# Load environment variables
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

print("Validating Qdrant credentials...")
print(f"QDRANT_URL: {QDRANT_URL}")

# Check if API key is valid
if not QDRANT_API_KEY or QDRANT_API_KEY == "YOUR_QDRANT_API_KEY_HERE":
    print("❌ QDRANT_API_KEY is not set or is a placeholder")
else:
    print("✅ QDRANT_API_KEY is set")

# Make a test request to Qdrant
if QDRANT_URL and QDRANT_API_KEY:
    try:
        # Format the URL properly
        if not QDRANT_URL.startswith(('http://', 'https://')):
            QDRANT_URL = f"https://{QDRANT_URL}"
        
        # Remove trailing slash if present
        QDRANT_URL = QDRANT_URL.rstrip('/')
        
        headers = {
            "api-key": QDRANT_API_KEY,
            "Content-Type": "application/json"
        }
        
        # Test request to get collections
        response = requests.get(f"{QDRANT_URL}/collections", headers=headers, timeout=10)
        
        if response.status_code == 200:
            print("✅ Successfully connected to Qdrant")
            collections_data = response.json()
            print(f"Available collections: {collections_data}")
        elif response.status_code == 403:
            print("❌ Forbidden: Invalid API key or insufficient permissions")
            print("Please verify your QDRANT_API_KEY is correct")
        elif response.status_code == 404:
            print("❌ Not Found: Invalid QDRANT_URL")
            print("Please verify your QDRANT_URL is correct")
        else:
            print(f"❌ Request failed with status code: {response.status_code}")
            print(f"Response: {response.text}")
    except requests.exceptions.ConnectionError:
        print("❌ Connection Error: Unable to connect to Qdrant")
        print("Please check your internet connection and QDRANT_URL")
    except requests.exceptions.Timeout:
        print("❌ Timeout: Request to Qdrant timed out")
        print("Please check your QDRANT_URL and internet connection")
    except Exception as e:
        print(f"❌ Error connecting to Qdrant: {e}")
else:
    print("❌ Cannot test connection: Missing QDRANT_URL or QDRANT_API_KEY")