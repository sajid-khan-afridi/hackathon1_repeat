#!/bin/bash
# Smoke Test Script for Phase 4B Staging Deployment
# Tests all critical endpoints and personalization features
# Usage: ./backend/scripts/smoke_test_staging.sh <staging-backend-url>

set -e

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration
STAGING_URL="${1:-https://hackathon1-staging.up.railway.app}"
RESULTS_FILE="smoke_test_results_$(date +%Y%m%d_%H%M%S).log"

# Test counters
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

# Helper functions
test_start() {
  TOTAL_TESTS=$((TOTAL_TESTS + 1))
  echo -e "${BLUE}[TEST $TOTAL_TESTS]${NC} $1"
}

test_pass() {
  PASSED_TESTS=$((PASSED_TESTS + 1))
  echo -e "  ${GREEN}✅ PASS${NC}: $1"
  echo "[PASS] $1" >> "$RESULTS_FILE"
}

test_fail() {
  FAILED_TESTS=$((FAILED_TESTS + 1))
  echo -e "  ${RED}❌ FAIL${NC}: $1"
  echo "[FAIL] $1" >> "$RESULTS_FILE"
}

test_skip() {
  echo -e "  ${YELLOW}⏭️  SKIP${NC}: $1"
  echo "[SKIP] $1" >> "$RESULTS_FILE"
}

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}Phase 4B Smoke Test - Staging${NC}"
echo -e "${BLUE}Target: $STAGING_URL${NC}"
echo -e "${BLUE}Date: $(date '+%Y-%m-%d %H:%M:%S')${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""

# Initialize results file
echo "Smoke Test Results - $(date)" > "$RESULTS_FILE"
echo "Target: $STAGING_URL" >> "$RESULTS_FILE"
echo "========================================" >> "$RESULTS_FILE"
echo "" >> "$RESULTS_FILE"

# ============================================
# Section 1: Infrastructure Health Checks
# ============================================
echo -e "${GREEN}Section 1: Infrastructure Health Checks${NC}"
echo ""

test_start "Backend /health endpoint responds"
if curl -s -f "$STAGING_URL/health" > /dev/null; then
  HEALTH_STATUS=$(curl -s "$STAGING_URL/health" | grep -o '"status":"ok"' || echo "")
  if [ -n "$HEALTH_STATUS" ]; then
    test_pass "Health endpoint returns status:ok"
  else
    test_fail "Health endpoint returned unexpected response"
  fi
else
  test_fail "Health endpoint not accessible"
fi

test_start "Backend /api/v1/health endpoint responds"
if curl -s -f "$STAGING_URL/api/v1/health" > /dev/null; then
  test_pass "Detailed health endpoint accessible"
else
  test_fail "Detailed health endpoint not accessible"
fi

test_start "API documentation endpoint (/api/docs)"
if curl -s -f "$STAGING_URL/api/docs" > /dev/null; then
  test_pass "API docs accessible"
else
  test_skip "API docs disabled in staging (expected for production-like setup)"
fi

test_start "CORS headers present"
CORS_HEADERS=$(curl -s -I -X OPTIONS "$STAGING_URL/health" | grep -i "access-control-allow-origin" || echo "")
if [ -n "$CORS_HEADERS" ]; then
  test_pass "CORS headers configured"
else
  test_fail "CORS headers missing"
fi

echo ""

# ============================================
# Section 2: Database Connectivity
# ============================================
echo -e "${GREEN}Section 2: Database Connectivity${NC}"
echo ""

test_start "Database connection via health endpoint"
DB_STATUS=$(curl -s "$STAGING_URL/api/v1/health" | grep -o '"database":"[^"]*"' | cut -d':' -f2 | tr -d '"' || echo "unknown")
if [ "$DB_STATUS" = "ok" ] || [ "$DB_STATUS" = "healthy" ]; then
  test_pass "Database connection healthy"
else
  test_fail "Database connection issue: $DB_STATUS"
fi

echo ""

# ============================================
# Section 3: Phase 4B Endpoints - Unauthenticated
# ============================================
echo -e "${GREEN}Section 3: Phase 4B Endpoints (Unauthenticated)${NC}"
echo ""

test_start "Skill level endpoint exists (GET /api/v1/skill-level)"
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" "$STAGING_URL/api/v1/skill-level")
if [ "$HTTP_CODE" = "401" ] || [ "$HTTP_CODE" = "200" ]; then
  test_pass "Endpoint exists (HTTP $HTTP_CODE)"
else
  test_fail "Unexpected HTTP code: $HTTP_CODE"
fi

test_start "Recommendations endpoint exists (GET /api/v1/recommendations)"
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" "$STAGING_URL/api/v1/recommendations")
if [ "$HTTP_CODE" = "401" ] || [ "$HTTP_CODE" = "200" ]; then
  test_pass "Endpoint exists (HTTP $HTTP_CODE)"
else
  test_fail "Unexpected HTTP code: $HTTP_CODE"
fi

test_start "Progress start endpoint exists (POST /api/v1/progress/start)"
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" -X POST "$STAGING_URL/api/v1/progress/start" \
  -H "Content-Type: application/json" \
  -d '{"chapter_id":"module-1/ros-intro"}')
if [ "$HTTP_CODE" = "401" ] || [ "$HTTP_CODE" = "422" ] || [ "$HTTP_CODE" = "200" ]; then
  test_pass "Endpoint exists (HTTP $HTTP_CODE)"
else
  test_fail "Unexpected HTTP code: $HTTP_CODE"
fi

test_start "Progress complete endpoint exists (POST /api/v1/progress/complete)"
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" -X POST "$STAGING_URL/api/v1/progress/complete" \
  -H "Content-Type: application/json" \
  -d '{"chapter_id":"module-1/ros-intro"}')
if [ "$HTTP_CODE" = "401" ] || [ "$HTTP_CODE" = "422" ] || [ "$HTTP_CODE" = "200" ]; then
  test_pass "Endpoint exists (HTTP $HTTP_CODE)"
else
  test_fail "Unexpected HTTP code: $HTTP_CODE"
fi

echo ""

# ============================================
# Section 4: Authentication Endpoints
# ============================================
echo -e "${GREEN}Section 4: Authentication Endpoints${NC}"
echo ""

test_start "Google OAuth initiation endpoint (GET /auth/google)"
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" -L "$STAGING_URL/auth/google")
if [ "$HTTP_CODE" = "302" ] || [ "$HTTP_CODE" = "200" ] || [ "$HTTP_CODE" = "307" ]; then
  test_pass "Google OAuth endpoint accessible (HTTP $HTTP_CODE)"
else
  test_fail "Google OAuth endpoint issue: HTTP $HTTP_CODE"
fi

test_start "GitHub OAuth initiation endpoint (GET /auth/github)"
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" -L "$STAGING_URL/auth/github")
if [ "$HTTP_CODE" = "302" ] || [ "$HTTP_CODE" = "200" ] || [ "$HTTP_CODE" = "307" ]; then
  test_pass "GitHub OAuth endpoint accessible (HTTP $HTTP_CODE)"
else
  test_fail "GitHub OAuth endpoint issue: HTTP $HTTP_CODE"
fi

echo ""

# ============================================
# Section 5: RAG Chatbot Endpoints
# ============================================
echo -e "${GREEN}Section 5: RAG Chatbot Endpoints${NC}"
echo ""

test_start "Chat query endpoint exists (POST /api/v1/query)"
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" -X POST "$STAGING_URL/api/v1/query" \
  -H "Content-Type: application/json" \
  -d '{"query":"What is ROS?","session_id":"test-session"}')
if [ "$HTTP_CODE" = "200" ] || [ "$HTTP_CODE" = "401" ] || [ "$HTTP_CODE" = "422" ]; then
  test_pass "Query endpoint accessible (HTTP $HTTP_CODE)"
else
  test_fail "Query endpoint issue: HTTP $HTTP_CODE"
fi

test_start "Chat sessions endpoint exists (GET /api/v1/chat/sessions)"
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" "$STAGING_URL/api/v1/chat/sessions")
if [ "$HTTP_CODE" = "200" ] || [ "$HTTP_CODE" = "401" ]; then
  test_pass "Sessions endpoint accessible (HTTP $HTTP_CODE)"
else
  test_fail "Sessions endpoint issue: HTTP $HTTP_CODE"
fi

echo ""

# ============================================
# Section 6: External Service Integrations
# ============================================
echo -e "${GREEN}Section 6: External Service Integrations${NC}"
echo ""

test_start "OpenAI integration status"
OPENAI_STATUS=$(curl -s "$STAGING_URL/api/v1/health" | grep -o '"openai":"[^"]*"' | cut -d':' -f2 | tr -d '"' || echo "unknown")
if [ "$OPENAI_STATUS" = "ok" ] || [ "$OPENAI_STATUS" = "healthy" ]; then
  test_pass "OpenAI integration healthy"
else
  test_fail "OpenAI integration issue: $OPENAI_STATUS"
fi

test_start "Qdrant integration status"
QDRANT_STATUS=$(curl -s "$STAGING_URL/api/v1/health" | grep -o '"qdrant":"[^"]*"' | cut -d':' -f2 | tr -d '"' || echo "unknown")
if [ "$QDRANT_STATUS" = "ok" ] || [ "$QDRANT_STATUS" = "healthy" ]; then
  test_pass "Qdrant integration healthy"
else
  test_fail "Qdrant integration issue: $QDRANT_STATUS"
fi

echo ""

# ============================================
# Section 7: Performance Tests
# ============================================
echo -e "${GREEN}Section 7: Performance Tests${NC}"
echo ""

test_start "Health endpoint response time < 200ms"
RESPONSE_TIME=$(curl -s -o /dev/null -w "%{time_total}" "$STAGING_URL/health")
RESPONSE_MS=$(echo "$RESPONSE_TIME * 1000" | bc | cut -d'.' -f1)
if [ "$RESPONSE_MS" -lt 200 ]; then
  test_pass "Response time: ${RESPONSE_MS}ms"
elif [ "$RESPONSE_MS" -lt 500 ]; then
  echo -e "  ${YELLOW}⚠️  WARN${NC}: Response time ${RESPONSE_MS}ms (acceptable but slow)"
else
  test_fail "Response time too slow: ${RESPONSE_MS}ms"
fi

test_start "API endpoint response time < 1000ms"
START_TIME=$(date +%s%3N)
curl -s -o /dev/null "$STAGING_URL/api/v1/health"
END_TIME=$(date +%s%3N)
RESPONSE_MS=$((END_TIME - START_TIME))
if [ "$RESPONSE_MS" -lt 1000 ]; then
  test_pass "Response time: ${RESPONSE_MS}ms"
elif [ "$RESPONSE_MS" -lt 2000 ]; then
  echo -e "  ${YELLOW}⚠️  WARN${NC}: Response time ${RESPONSE_MS}ms (acceptable but slow)"
else
  test_fail "Response time too slow: ${RESPONSE_MS}ms"
fi

echo ""

# ============================================
# Section 8: Security Headers
# ============================================
echo -e "${GREEN}Section 8: Security Headers${NC}"
echo ""

test_start "HTTPS redirect (if applicable)"
if [[ "$STAGING_URL" == https://* ]]; then
  test_pass "Using HTTPS"
else
  test_fail "Not using HTTPS (insecure)"
fi

test_start "Security headers present"
SECURITY_HEADERS=$(curl -s -I "$STAGING_URL/health" | grep -iE "(X-Frame-Options|X-Content-Type-Options|Strict-Transport-Security)" || echo "")
if [ -n "$SECURITY_HEADERS" ]; then
  test_pass "Security headers configured"
else
  test_skip "Security headers not found (may be optional for staging)"
fi

echo ""

# ============================================
# Test Summary
# ============================================
echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}Test Summary${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""
echo -e "Total Tests:  ${BLUE}$TOTAL_TESTS${NC}"
echo -e "Passed:       ${GREEN}$PASSED_TESTS${NC}"
echo -e "Failed:       ${RED}$FAILED_TESTS${NC}"
echo -e "Skipped:      ${YELLOW}$((TOTAL_TESTS - PASSED_TESTS - FAILED_TESTS))${NC}"
echo ""

# Calculate pass rate
if [ "$TOTAL_TESTS" -gt 0 ]; then
  PASS_RATE=$((PASSED_TESTS * 100 / TOTAL_TESTS))
  echo -e "Pass Rate:    ${GREEN}${PASS_RATE}%${NC}"
else
  PASS_RATE=0
fi

# Write summary to results file
echo "" >> "$RESULTS_FILE"
echo "========================================" >> "$RESULTS_FILE"
echo "Summary:" >> "$RESULTS_FILE"
echo "  Total Tests: $TOTAL_TESTS" >> "$RESULTS_FILE"
echo "  Passed: $PASSED_TESTS" >> "$RESULTS_FILE"
echo "  Failed: $FAILED_TESTS" >> "$RESULTS_FILE"
echo "  Pass Rate: ${PASS_RATE}%" >> "$RESULTS_FILE"

echo ""
echo -e "${BLUE}Results saved to: $RESULTS_FILE${NC}"
echo ""

# Exit with appropriate code
if [ "$FAILED_TESTS" -gt 0 ]; then
  echo -e "${RED}❌ Smoke tests FAILED${NC}"
  echo -e "${YELLOW}Review failures before proceeding to UAT${NC}"
  exit 1
elif [ "$PASS_RATE" -lt 80 ]; then
  echo -e "${YELLOW}⚠️  Smoke tests passed with warnings${NC}"
  echo -e "${YELLOW}Review results before proceeding to UAT${NC}"
  exit 0
else
  echo -e "${GREEN}✅ All smoke tests PASSED${NC}"
  echo -e "${GREEN}Ready for UAT${NC}"
  exit 0
fi
