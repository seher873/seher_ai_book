#!/bin/bash

echo "1) Checking if uvicorn is running"
ps aux | grep uvicorn

echo ""
echo "2) Starting backend server (if not running)"
pkill -f uvicorn  # Kill any existing uvicorn processes
uvicorn final_server:app --host 127.0.0.1 --port 8000 --reload &
sleep 3

echo ""
echo "3) Checking available API routes"
echo "Available endpoints:"
curl -s http://127.0.0.1:8000/openapi.json | grep -o '"[^"]*"' | grep -E '^"/'

echo ""
echo "4) Health check"
curl http://127.0.0.1:8000/health

echo ""
echo "5) Testing query endpoint with book question"
curl -X POST http://127.0.0.1:8000/query \
-H "Content-Type: application/json" \
-d '{"query": "What is Physical AI?"}'

echo -e "\n\n6) Testing query endpoint with another question"
curl -X POST http://127.0.0.1:8000/query \
-H "Content-Type: application/json" \
-d '{"query": "Explain machine learning"}'

echo -e "\n\n7) Testing query endpoint with a third question"
curl -X POST http://127.0.0.1:8000/query \
-H "Content-Type: application/json" \
-d '{"query": "What are neural networks?"}'

echo -e "\n\n8) Testing the specific example from the instructions"
curl -X POST http://127.0.0.1:8000/query \
-H "Content-Type: application/json" \
-d '{"query": "What are humanoid robots?"}'