#!/bin/bash
# =============================================================================
# Phase 4B Personalization Engine - Smoke Test Script
# =============================================================================
# Purpose: Automated smoke tests for staging deployment validation
# Usage: bash specs/1-personalization-engine/scripts/smoke_test.sh
# Prerequisites: curl, jq installed
# =============================================================================

set -e  # Exit on error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
STAGING_API="${STAGING_API:-https://hackathon1-staging.up.railway.app/api/v1}"
TIMEOUT=10
PASSED=0
FAILED=0

echo "=========================================="
echo "Phase 4B Smoke Tests - Staging Deployment"
echo "=========================================="
echo "API Endpoint: $STAGING_API"
echo "Timeout: ${TIMEOUT}s"
echo ""

# Helper function for test assertions
test_endpoint() {
    local test_name="$1"
    local method="$2"
    local endpoint="$3"
    local data="$4"
    local expected_status="$5"

    echo -n "Testing: $test_name ... "

    if [ "$method" == "GET" ]; then
        response=$(curl -s -w "\n%{http_code}" --max-time $TIMEOUT "$STAGING_API$endpoint" || echo "000")
    else
        response=$(curl -s -w "\n%{http_code}" --max-time $TIMEOUT -X "$method" \
            -H "Content-Type: application/json" \
            -d "$data" \
            "$STAGING_API$endpoint" || echo "000")
    fi

    http_code=$(echo "$response" | tail -n1)
    body=$(echo "$response" | head -n-1)

    if [ "$http_code" == "$expected_status" ]; then
        echo -e "${GREEN}PASS${NC} (HTTP $http_code)"
        ((PASSED++))
        return 0
    else
        echo -e "${RED}FAIL${NC} (Expected HTTP $expected_status, got $http_code)"
        echo "Response: $body"
        ((FAILED++))
        return 1
    fi
}

# =============================================================================
# Test Suite 1: Infrastructure Health
# =============================================================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test Suite 1: Infrastructure Health"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# T001: Health endpoint
test_endpoint \
    "Health Check" \
    "GET" \
    "/../health" \
    "" \
    "200"

# T002: API docs accessible
echo -n "Testing: API Documentation ... "
docs_response=$(curl -s --max-time $TIMEOUT "$STAGING_API/../docs" || echo "")
if [[ "$docs_response" == *"FastAPI"* ]]; then
    echo -e "${GREEN}PASS${NC}"
    ((PASSED++))
else
    echo -e "${RED}FAIL${NC}"
    ((FAILED++))
fi

echo ""

# =============================================================================
# Test Suite 2: Phase 4B - Personalization Endpoints (Unauthenticated)
# =============================================================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test Suite 2: Personalization (Unauthenticated)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# T010: Skill level endpoint should require auth
test_endpoint \
    "Skill Level (Unauthenticated)" \
    "GET" \
    "/skill-level" \
    "" \
    "401"

# T011: Recommendations endpoint should require auth
test_endpoint \
    "Recommendations (Unauthenticated)" \
    "GET" \
    "/recommendations" \
    "" \
    "401"

# T012: Progress tracking should require auth
test_endpoint \
    "Progress Start (Unauthenticated)" \
    "POST" \
    "/progress/start" \
    '{"chapter_id":"module-1/ros-intro"}' \
    "401"

echo ""

# =============================================================================
# Test Suite 3: RAG Chatbot (With Personalization)
# =============================================================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test Suite 3: RAG Chatbot"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# T020: RAG chatbot query (unauthenticated - should use default profile)
test_endpoint \
    "RAG Query (Default Profile)" \
    "POST" \
    "/chat/query" \
    '{"query":"What is ROS 2?","session_id":"smoke-test-'$(date +%s)'"}' \
    "200"

# T021: RAG chatbot query with context
test_endpoint \
    "RAG Query (With Context)" \
    "POST" \
    "/chat/query" \
    '{"query":"How do I create a publisher?","session_id":"smoke-test-context-'$(date +%s)'","context":"I am learning ROS 2"}' \
    "200"

echo ""

# =============================================================================
# Test Suite 4: Database Connection Validation
# =============================================================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test Suite 4: Database Validation"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# T030: Check if chapter metadata exists
echo -n "Testing: Chapter Metadata Exists ... "
chapter_response=$(curl -s --max-time $TIMEOUT "$STAGING_API/chat/query" \
    -X POST \
    -H "Content-Type: application/json" \
    -d '{"query":"List available chapters","session_id":"smoke-metadata-'$(date +%s)'"}' || echo "")

if [[ "$chapter_response" == *"module"* ]] || [[ "$chapter_response" == *"chapter"* ]]; then
    echo -e "${GREEN}PASS${NC}"
    ((PASSED++))
else
    echo -e "${YELLOW}WARN${NC} (Could not verify chapter metadata)"
fi

echo ""

# =============================================================================
# Test Suite 5: CORS Configuration
# =============================================================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test Suite 5: CORS Configuration"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# T040: CORS headers present
echo -n "Testing: CORS Headers ... "
cors_response=$(curl -s -I -H "Origin: https://staging-hackathon1.vercel.app" \
    --max-time $TIMEOUT "$STAGING_API/../health" || echo "")

if [[ "$cors_response" == *"access-control-allow-origin"* ]]; then
    echo -e "${GREEN}PASS${NC}"
    ((PASSED++))
else
    echo -e "${RED}FAIL${NC}"
    ((FAILED++))
fi

echo ""

# =============================================================================
# Test Suite 6: Performance Validation
# =============================================================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test Suite 6: Performance"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# T050: Response time under 2 seconds
echo -n "Testing: Response Time (< 2s) ... "
start_time=$(date +%s%N)
curl -s --max-time $TIMEOUT "$STAGING_API/../health" > /dev/null
end_time=$(date +%s%N)
elapsed_ms=$(( (end_time - start_time) / 1000000 ))

if [ $elapsed_ms -lt 2000 ]; then
    echo -e "${GREEN}PASS${NC} (${elapsed_ms}ms)"
    ((PASSED++))
else
    echo -e "${YELLOW}WARN${NC} (${elapsed_ms}ms - slower than expected)"
fi

echo ""

# =============================================================================
# Test Results Summary
# =============================================================================
echo "=========================================="
echo "Smoke Test Results"
echo "=========================================="
echo -e "Passed:  ${GREEN}$PASSED${NC}"
echo -e "Failed:  ${RED}$FAILED${NC}"
echo "Total:   $((PASSED + FAILED))"
echo ""

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}✓ All smoke tests passed!${NC}"
    echo "Staging deployment is healthy and ready for UAT."
    exit 0
else
    echo -e "${RED}✗ Some smoke tests failed!${NC}"
    echo "Review failed tests and fix issues before proceeding to UAT."
    exit 1
fi
