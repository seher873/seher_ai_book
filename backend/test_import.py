#!/usr/bin/env python3
"""
Test script to check if main.py can be imported without errors
"""
import sys
import os
sys.path.insert(0, '/mnt/c/Users/user/Desktop/my_ai_book/backend')

# Change to the backend directory
os.chdir('/mnt/c/Users/user/Desktop/my_ai_book/backend')

try:
    print("Attempting to import main.py...")
    from main import app
    print("Import successful!")
    print(f"App: {app}")
    
    # Try to access the environment variables
    from dotenv import load_dotenv
    import os
    load_dotenv()
    
    print(f"OPENROUTER_API_KEY: {'SET' if os.getenv('OPENROUTER_API_KEY') else 'NOT SET'}")
    print(f"OPENROUTER_MODEL: {os.getenv('OPENROUTER_MODEL', 'xiaomi/mimo-v2-flash')}")
    
    # Try to initialize the OpenAI client
    import openai
    openai_client = openai.OpenAI(
        api_key=os.getenv("OPENROUTER_API_KEY"),
        base_url="https://openrouter.ai/api/v1",
    )
    print("OpenAI client initialized successfully!")
    
    print("All tests passed!")
    
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()