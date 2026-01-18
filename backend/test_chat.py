import requests
import json
import time

def test_chat():
    url = "http://localhost:8000/chat"
    payload = {
        "query": "What are the core concepts of Physical AI covered in the book?",
        "max_results": 3
    }

    print(f"Sending query to {url}: {payload['query']}")
    try:
        response = requests.post(url, json=payload)
        if response.status_code == 200:
            print("\nChat Response received perfectly:")
            data = response.json()
            print(f"Response: {data['response']}")
            print(f"\nSources used: {len(data['sources'])}")
            for src in data['sources']:
                print(f"- {src['title']} (Score: {src['score']:.4f})")
        else:
            print(f"Error: {response.status_code}")
            print(response.text)
    except Exception as e:
        print(f"\nConnection failed: {e}")
        print("Note: Make sure the backend server (main.py) is running on port 8000.")

if __name__ == "__main__":
    test_chat()
