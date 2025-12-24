import os
from dotenv import load_dotenv

# Explicitly load the .env file
env_path = os.path.join(os.path.dirname(__file__), '.env')
print(f"Looking for .env file at: {env_path}")
print(f"Does .env file exist? {os.path.exists(env_path)}")

# Load the environment variables
load_dotenv(dotenv_path=env_path)

print("\nAfter loading .env file:")
print(f"QDRANT_URL: {os.getenv('QDRANT_URL')}")
print(f"QDRANT_API_KEY: {os.getenv('QDRANT_API_KEY')}")
print(f"COHERE_API_KEY: {os.getenv('COHERE_API_KEY')}")
print(f"OPENROUTER_API_KEY: {os.getenv('OPENROUTER_API_KEY')}")

# Read and display the contents of the .env file
if os.path.exists(env_path):
    print("\nContents of .env file:")
    with open(env_path, 'r') as file:
        print(file.read())
else:
    print("\n.env file does not exist at the expected location")

# Try to load environment variables from the system as well
print("\nChecking if variables are set in the system environment:")
print(f"QDRANT_URL in os.environ: {'QDRANT_URL' in os.environ}")
print(f"QDRANT_API_KEY in os.environ: {'QDRANT_API_KEY' in os.environ}")
print(f"COHERE_API_KEY in os.environ: {'COHERE_API_KEY' in os.environ}")