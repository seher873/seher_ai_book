import os
import requests
from dotenv import load_dotenv

load_dotenv()

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

headers = {
    "api-key": QDRANT_API_KEY,
    "Content-Type": "application/json"
}

# Check collections
response = requests.get(f"{QDRANT_URL}/collections", headers=headers, timeout=10)
print("Collections:", response.json())

# Check book_docs collection
response = requests.get(f"{QDRANT_URL}/collections/book_docs", headers=headers, timeout=10)
print("\nbook_docs collection info:", response.json())

# Check seher_robotic_book_netlify_app collection
response = requests.get(f"{QDRANT_URL}/collections/seher_robotic_book_netlify_app", headers=headers, timeout=10)
if response.status_code == 200:
    print("\nseher_robotic_book_netlify_app collection info:", response.json())
else:
    print("\nseher_robotic_book_netlify_app collection does not exist")
