#!/bin/bash

# Railway Deployment Quick Test Script
# Usage: ./test_railway_deployment.sh https://your-app.up.railway.app

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Check if URL provided
if [ -z "$1" ]; then
    echo -e "${RED}Error: Railway URL required${NC}"
    echo "Usage: $0 <railway-url>"
    echo "Example: $0 https://your-app.up.railway.app"
    exit 1
fi

RAILWAY_URL="${1%/}"  # Remove trailing slash

echo -e "\n${BOLD}${BLUE}==========================================${NC}"
echo -e "${BOLD}${BLUE}Railway Deployment Quick Test${NC}"
echo -e "${BOLD}${BLUE}==========================================${NC}\n"
echo -e "Testing: ${RAILWAY_URL}\n"

# Test counter
PASSED=0
FAILED=0

# Function to test endpoint
test_endpoint() {
    local name="$1"
    local url="$2"
    local expected_status="$3"

    echo -n "Testing ${name}... "

    response=$(curl -s -o /dev/null -w "%{http_code}" --max-time 30 "${url}")

    if [ "$response" == "$expected_status" ]; then
        echo -e "${GREEN}✓ PASS${NC} (Status: ${response})"
        ((PASSED++))
    else
        echo -e "${RED}✗ FAIL${NC} (Expected: ${expected_status}, Got: ${response})"
        ((FAILED++))
    fi
}

# Function to test endpoint with JSON response
test_json_endpoint() {
    local name="$1"
    local url="$2"

    echo -e "\nTesting ${BOLD}${name}${NC}..."

    response=$(curl -s --max-time 30 "${url}")
    status=$?

    if [ $status -eq 0 ]; then
        echo -e "${GREEN}✓ PASS${NC} - Response received"
        echo "$response" | jq '.' 2>/dev/null || echo "$response"
        ((PASSED++))
    else
        echo -e "${RED}✗ FAIL${NC} - Request failed"
        ((FAILED++))
    fi
}

# Function to test query endpoint
test_query() {
    local name="$1"
    local url="$2"

    echo -e "\nTesting ${BOLD}${name}${NC}..."

    response=$(curl -s --max-time 45 -X POST "${url}" \
        -H "Content-Type: application/json" \
        -d '{
            "query": "What is ROS?",
            "filters": {},
            "top_k": 3
        }')

    status=$?

    if [ $status -eq 0 ]; then
        # Check if response contains expected fields
        if echo "$response" | jq -e '.answer' > /dev/null 2>&1; then
            echo -e "${GREEN}✓ PASS${NC} - Query successful"

            # Extract key info
            confidence=$(echo "$response" | jq -r '.confidence')
            sources=$(echo "$response" | jq '.sources | length')
            session_id=$(echo "$response" | jq -r '.session_id')

            echo "  Confidence: ${confidence}"
            echo "  Sources: ${sources}"
            echo "  Session ID: ${session_id}"

            # Save session ID for later tests
            echo "$session_id" > /tmp/railway_test_session_id.txt

            ((PASSED++))
        else
            echo -e "${RED}✗ FAIL${NC} - Invalid response format"
            echo "$response"
            ((FAILED++))
        fi
    else
        echo -e "${RED}✗ FAIL${NC} - Request failed"
        ((FAILED++))
    fi
}

# Function to test CORS
test_cors() {
    echo -e "\nTesting ${BOLD}CORS Configuration${NC}..."

    response=$(curl -s -I -X OPTIONS "${RAILWAY_URL}/api/v1/health" \
        -H "Origin: https://sajid-khan-afridi.github.io" \
        -H "Access-Control-Request-Method: GET")

    if echo "$response" | grep -i "access-control-allow-origin" > /dev/null; then
        echo -e "${GREEN}✓ PASS${NC} - CORS headers present"
        echo "$response" | grep -i "access-control"
        ((PASSED++))
    else
        echo -e "${RED}✗ FAIL${NC} - CORS headers missing"
        ((FAILED++))
    fi
}

# Run tests
echo -e "${BOLD}Core Health Tests${NC}"
echo "=================================="
test_endpoint "Root Endpoint" "${RAILWAY_URL}/" "200"
test_endpoint "Simple Health" "${RAILWAY_URL}/health" "200"

# Detailed health can return 200 or 503 (if services degraded)
echo -n "Testing Detailed Health... "
response=$(curl -s -o /dev/null -w "%{http_code}" --max-time 30 "${RAILWAY_URL}/api/v1/health")
if [ "$response" == "200" ] || [ "$response" == "503" ]; then
    echo -e "${GREEN}✓ PASS${NC} (Status: ${response})"
    ((PASSED++))
else
    echo -e "${RED}✗ FAIL${NC} (Status: ${response})"
    ((FAILED++))
fi

test_endpoint "Query Health" "${RAILWAY_URL}/api/v1/query/health" "200"

# CORS test
echo -e "\n${BOLD}CORS Configuration${NC}"
echo "=================================="
test_cors

# Query test
echo -e "\n${BOLD}Query Endpoint Tests${NC}"
echo "=================================="
test_query "RAG Query Endpoint" "${RAILWAY_URL}/api/v1/query"

# Session test (if we have a session ID)
if [ -f /tmp/railway_test_session_id.txt ]; then
    SESSION_ID=$(cat /tmp/railway_test_session_id.txt)

    echo -e "\n${BOLD}Session Management Tests${NC}"
    echo "=================================="

    echo -e "\nTesting ${BOLD}Session Retrieval${NC}..."
    response=$(curl -s -o /dev/null -w "%{http_code}" --max-time 30 \
        "${RAILWAY_URL}/api/v1/chat/sessions/${SESSION_ID}")

    if [ "$response" == "200" ]; then
        echo -e "${GREEN}✓ PASS${NC} (Status: ${response})"
        ((PASSED++))
    else
        echo -e "${RED}✗ FAIL${NC} (Status: ${response})"
        ((FAILED++))
    fi

    # Clean up
    rm /tmp/railway_test_session_id.txt
fi

# Print summary
echo -e "\n${BOLD}${BLUE}==========================================${NC}"
echo -e "${BOLD}${BLUE}Test Summary${NC}"
echo -e "${BOLD}${BLUE}==========================================${NC}\n"

TOTAL=$((PASSED + FAILED))
PASS_RATE=$((PASSED * 100 / TOTAL))

echo "Total Tests: ${TOTAL}"
echo -e "${GREEN}Passed: ${PASSED}${NC}"
echo -e "${RED}Failed: ${FAILED}${NC}"
echo "Pass Rate: ${PASS_RATE}%"

echo ""

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}${BOLD}All tests passed! ✓${NC}"
    exit 0
else
    echo -e "${YELLOW}${BOLD}Some tests failed. Check the output above.${NC}"
    exit 1
fi
