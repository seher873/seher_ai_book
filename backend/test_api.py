import requests
import json

# Test the health endpoint
try:
    response = requests.get("http://0.0.0.0:8001/health")
    print(f"Health check response: {response.json()}")
except Exception as e:
    print(f"Health check failed: {e}")

# Test the query endpoint with a sample query
try:
    sample_query = {"query": "What is artificial intelligence?"}
    response = requests.post(
        "http://0.0.0.0:8001/query",
        data=json.dumps(sample_query),
        headers={"Content-Type": "application/json"}
    )
    print(f"Query response: {response.json()}")
except Exception as e:
    print(f"Query test failed: {e}")

print("\nMake sure the server is running before executing this test.")
print("If the server isn't running, execute:")
print("cd /mnt/c/Users/user/Desktop/my_ai_book/backend")
print("python final_server.py")