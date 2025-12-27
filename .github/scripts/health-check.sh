#!/bin/bash

# Health Check Script for Deployment Pipeline
# Verifies service availability with retry logic and JSON response validation
# Usage: ./health-check.sh <url> [max_retries] [timeout]

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
URL="${1:-}"
MAX_RETRIES="${2:-3}"
TIMEOUT="${3:-10}"
RETRY_DELAY=5  # seconds between retries

# Validate arguments
if [ -z "$URL" ]; then
    echo -e "${RED}Error: URL is required${NC}"
    echo "Usage: $0 <url> [max_retries] [timeout]"
    exit 1
fi

echo "========================================="
echo "Health Check Starting"
echo "========================================="
echo "URL: $URL"
echo "Max Retries: $MAX_RETRIES"
echo "Timeout: ${TIMEOUT}s"
echo "========================================="

# Function to check if URL is responding
check_health() {
    local url=$1
    local timeout=$2

    echo -e "${YELLOW}Attempting health check...${NC}"

    # Make HTTP request with timeout
    response=$(curl -s -w "\n%{http_code}\n%{time_total}" \
        --max-time "$timeout" \
        --fail \
        "$url" 2>&1) || return 1

    # Extract status code and response time
    status_code=$(echo "$response" | tail -n 2 | head -n 1)
    response_time=$(echo "$response" | tail -n 1)
    body=$(echo "$response" | head -n -2)

    echo "Status Code: $status_code"
    echo "Response Time: ${response_time}s"

    # Check status code
    if [ "$status_code" != "200" ]; then
        echo -e "${RED}Failed: HTTP status $status_code${NC}"
        return 1
    fi

    # Check if response time is acceptable (<2 seconds)
    if (( $(echo "$response_time > 2.0" | bc -l) )); then
        echo -e "${YELLOW}Warning: Response time ${response_time}s exceeds 2s threshold${NC}"
    fi

    # Validate JSON response if it's a JSON endpoint
    if echo "$body" | jq . >/dev/null 2>&1; then
        echo "Valid JSON response detected"

        # Check for specific health indicators if it's a /health endpoint
        if [[ "$url" == *"/health"* ]]; then
            status=$(echo "$body" | jq -r '.status // empty')
            if [ -n "$status" ]; then
                echo "Health Status: $status"
                if [ "$status" != "healthy" ]; then
                    echo -e "${RED}Failed: Service reports unhealthy status${NC}"
                    return 1
                fi
            fi

            # Check dependent services if present
            services=$(echo "$body" | jq -r '.services // empty')
            if [ -n "$services" ]; then
                echo "Dependent Services:"
                echo "$body" | jq -r '.services | to_entries[] | "  \(.key): \(.value)"'

                # Check if any service is down
                down_services=$(echo "$body" | jq -r '.services | to_entries[] | select(.value == "down") | .key')
                if [ -n "$down_services" ]; then
                    echo -e "${RED}Failed: Services down: $down_services${NC}"
                    return 1
                fi
            fi
        fi
    else
        # For HTML responses, check if body is not empty
        if [ -z "$body" ]; then
            echo -e "${RED}Failed: Empty response body${NC}"
            return 1
        fi
        echo "HTML response received (${#body} bytes)"
    fi

    return 0
}

# Exponential backoff retry logic
attempt=1
while [ $attempt -le $MAX_RETRIES ]; do
    echo ""
    echo "----------------------------------------"
    echo "Attempt $attempt of $MAX_RETRIES"
    echo "----------------------------------------"

    if check_health "$URL" "$TIMEOUT"; then
        echo ""
        echo -e "${GREEN}=========================================${NC}"
        echo -e "${GREEN}✓ Health Check PASSED${NC}"
        echo -e "${GREEN}=========================================${NC}"
        exit 0
    else
        if [ $attempt -lt $MAX_RETRIES ]; then
            # Calculate backoff delay (exponential: 5, 10, 20 seconds)
            backoff_delay=$((RETRY_DELAY * (2 ** (attempt - 1))))
            echo -e "${YELLOW}Retry in ${backoff_delay}s...${NC}"
            sleep "$backoff_delay"
        fi
    fi

    attempt=$((attempt + 1))
done

# All retries failed
echo ""
echo -e "${RED}=========================================${NC}"
echo -e "${RED}✗ Health Check FAILED${NC}"
echo -e "${RED}All $MAX_RETRIES attempts failed${NC}"
echo -e "${RED}=========================================${NC}"
exit 1
