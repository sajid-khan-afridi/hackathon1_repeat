# Railway Deployment Testing Guide

This guide provides comprehensive testing procedures for your RAG Chatbot API deployed on Railway.

## Table of Contents

1. [Pre-Deployment Checklist](#pre-deployment-checklist)
2. [Environment Variables](#environment-variables)
3. [Testing Methods](#testing-methods)
4. [Post-Deployment Verification](#post-deployment-verification)
5. [Troubleshooting](#troubleshooting)

---

## Pre-Deployment Checklist

Before deploying to Railway, ensure:

- [ ] All environment variables are configured in Railway dashboard
- [ ] CORS origins include your frontend URL and Railway backend URL
- [ ] Database (Neon PostgreSQL) is accessible
- [ ] Vector store (Qdrant) is accessible
- [ ] OpenAI API key is valid
- [ ] `backend/railway.json` is properly configured
- [ ] `backend/requirements.txt` includes all dependencies
- [ ] Code is committed and pushed to main branch

---

## Environment Variables

Ensure these are set in your Railway project:

### Required Variables

```bash
# API Keys
OPENAI_API_KEY=sk-...
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION=robotics_textbook
DATABASE_URL=postgres://user:password@ep-your-endpoint.neon.tech/dbname?sslmode=require

# Application Configuration
ENVIRONMENT=production
CORS_ORIGINS=https://sajid-khan-afridi.github.io,https://your-railway-app.up.railway.app

# Rate Limiting
RATE_LIMIT_ANONYMOUS=10
RATE_LIMIT_AUTHENTICATED=50

# RAG Configuration
CONFIDENCE_THRESHOLD=0.2
TOP_K_CHUNKS=5
MAX_CONVERSATION_HISTORY=5
SESSION_RETENTION_DAYS=30
```

### CORS Configuration

**IMPORTANT**: Your `CORS_ORIGINS` must include:
1. Your frontend domain (e.g., `https://sajid-khan-afridi.github.io`)
2. Your Railway backend URL (e.g., `https://your-app.up.railway.app`)
3. Any other domains that need access

Multiple origins should be comma-separated with **no spaces**:
```bash
CORS_ORIGINS=https://domain1.com,https://domain2.com,http://localhost:3000
```

---

## Testing Methods

We provide three different testing methods:

### Method 1: Python Testing Suite (Recommended)

**Most comprehensive** - Tests all endpoints, streaming, CORS, rate limiting, and session management.

```bash
# Install dependencies
pip install httpx

# Run tests
python test_railway_deployment.py https://your-app.up.railway.app

# View results
cat deployment_test_results.json
```

**Features:**
- Tests all API endpoints
- Validates streaming responses
- Checks CORS configuration
- Tests session management
- Generates JSON report
- Color-coded output

---

### Method 2: Bash Script (Quick Test)

**Fastest** - Requires only curl and jq (optional).

```bash
# Make script executable
chmod +x test_railway_deployment.sh

# Run tests
./test_railway_deployment.sh https://your-app.up.railway.app
```

**Features:**
- Quick health checks
- CORS verification
- Query endpoint test
- Session retrieval test
- No dependencies (except curl)

---

### Method 3: Browser Testing (Frontend Test)

**Most realistic** - Tests from actual browser environment with CORS.

1. Open `test_railway_frontend.html` in your browser
2. Enter your Railway URL
3. Click "Run All Health Tests"
4. Test query endpoints (both regular and streaming)
5. Verify CORS configuration

**Features:**
- Real browser CORS testing
- Interactive UI
- Streaming response visualization
- Save API URL for future tests

---

## Post-Deployment Verification

### Step 1: Basic Health Check

```bash
# Replace with your Railway URL
export RAILWAY_URL="https://your-app.up.railway.app"

# Test simple health endpoint
curl $RAILWAY_URL/health

# Expected: {"status": "ok"}
```

### Step 2: Detailed Health Check

```bash
# Test detailed health endpoint
curl $RAILWAY_URL/api/v1/health | jq '.'
```

**Expected Response:**
```json
{
  "status": "healthy",
  "database": "healthy",
  "vector_store": "healthy",
  "llm": "healthy",
  "details": {
    "database": "Connected successfully",
    "vector_store": "Collection 'robotics_textbook' found",
    "llm": "OpenAI API accessible"
  }
}
```

**Possible Statuses:**
- `healthy` - All services operational
- `degraded` - Some non-critical services unavailable
- `unhealthy` - Critical services down (returns 503)

### Step 3: Test CORS Headers

```bash
# Test CORS from command line
curl -I -X OPTIONS $RAILWAY_URL/api/v1/health \
  -H "Origin: https://sajid-khan-afridi.github.io" \
  -H "Access-Control-Request-Method: GET"
```

**Look for these headers:**
```
access-control-allow-origin: https://sajid-khan-afridi.github.io
access-control-allow-credentials: true
access-control-allow-methods: GET, POST, PUT, DELETE, OPTIONS
access-control-allow-headers: Content-Type, Authorization, X-Correlation-ID, X-Session-Token
```

### Step 4: Test Query Endpoint

```bash
# Send a test query
curl -X POST $RAILWAY_URL/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS?",
    "filters": {},
    "top_k": 3
  }' | jq '.'
```

**Expected Response:**
```json
{
  "answer": "ROS (Robot Operating System) is...",
  "sources": [
    {
      "chapter_id": "module-1-chapter-1",
      "chapter_title": "Introduction to ROS",
      "relevance_score": 0.92,
      "excerpt": "...",
      "position": 1
    }
  ],
  "confidence": 0.89,
  "session_id": "123e4567-e89b-12d3-a456-426614174000",
  "tokens_used": {
    "input_tokens": 245,
    "output_tokens": 127,
    "total_tokens": 372
  }
}
```

### Step 5: Test Streaming Response

```bash
# Test streaming endpoint
curl -X POST $RAILWAY_URL/api/v1/query \
  -H "Content-Type: application/json" \
  -H "Accept: text/event-stream" \
  -d '{
    "query": "What is a robot?",
    "filters": {},
    "top_k": 3
  }'
```

**Expected Output (SSE stream):**
```
data: {"chunk": "A", "done": false}

data: {"chunk": " robot", "done": false}

data: {"chunk": " is", "done": false}

data: {"chunk": "", "done": true, "sources": [...], "confidence": 0.89, "session_id": "..."}
```

### Step 6: Test from Frontend

Open your frontend application and:

1. **Test Query**: Send a question through your chatbot UI
2. **Check CORS**: Verify no CORS errors in browser console (F12)
3. **Verify Response**: Ensure answers are displayed correctly
4. **Test Streaming**: Verify streaming responses work smoothly
5. **Check Citations**: Ensure source citations are displayed

---

## API Endpoints Reference

### Health Endpoints

| Endpoint | Method | Description | Success Status |
|----------|--------|-------------|----------------|
| `/` | GET | Root endpoint | 200 |
| `/health` | GET | Simple health check | 200 |
| `/api/v1/health` | GET | Detailed health with dependencies | 200 or 503 |
| `/api/v1/query/health` | GET | Query service components health | 200 |

### Query Endpoints

| Endpoint | Method | Description | Headers |
|----------|--------|-------------|---------|
| `/api/v1/query` | POST | RAG query (non-streaming) | `Content-Type: application/json` |
| `/api/v1/query` | POST | RAG query (streaming) | `Accept: text/event-stream` |

### Session Endpoints

| Endpoint | Method | Description | Success Status |
|----------|--------|-------------|----------------|
| `/api/v1/chat/sessions/{id}` | GET | Retrieve session history | 200 |
| `/api/v1/chat/sessions/{id}` | DELETE | Delete session | 204 |

---

## Troubleshooting

### Issue: CORS Errors in Browser Console

**Symptoms:**
```
Access to fetch at 'https://your-app.up.railway.app/api/v1/query'
from origin 'https://sajid-khan-afridi.github.io' has been blocked by CORS policy
```

**Solution:**
1. Check Railway environment variables:
   ```bash
   # Should include your frontend domain
   CORS_ORIGINS=https://sajid-khan-afridi.github.io,https://your-app.up.railway.app
   ```

2. Verify CORS middleware is configured (backend/app/main.py:30)

3. Test CORS headers:
   ```bash
   curl -I -X OPTIONS $RAILWAY_URL/api/v1/health \
     -H "Origin: https://sajid-khan-afridi.github.io"
   ```

4. Restart Railway service after updating environment variables

---

### Issue: 503 Service Unavailable

**Symptoms:**
```json
{
  "detail": "Critical services unavailable"
}
```

**Solution:**
1. Check detailed health endpoint:
   ```bash
   curl $RAILWAY_URL/api/v1/health
   ```

2. Verify each service status:
   - **Database unhealthy**: Check `DATABASE_URL` is correct and Neon is accessible
   - **Vector store unhealthy**: Verify `QDRANT_URL` and `QDRANT_API_KEY`
   - **LLM unhealthy**: Verify `OPENAI_API_KEY` is valid

3. Check Railway logs for detailed error messages

---

### Issue: Query Timeout

**Symptoms:**
```json
{
  "detail": "Request timeout - the query took too long to process"
}
```

**Solution:**
1. Check OpenAI API status: https://status.openai.com
2. Reduce `TOP_K_CHUNKS` to improve performance
3. Check Qdrant vector store performance
4. Verify network latency between Railway and external services

---

### Issue: Rate Limiting

**Symptoms:**
```json
{
  "detail": "Rate limit exceeded"
}
```

**Solution:**
1. Check rate limit headers:
   ```bash
   curl -I $RAILWAY_URL/api/v1/query
   ```

2. Adjust rate limits in Railway environment:
   ```bash
   RATE_LIMIT_ANONYMOUS=20  # Increase if needed
   RATE_LIMIT_AUTHENTICATED=100
   ```

3. Implement authentication for higher limits

---

### Issue: Database Connection Errors

**Symptoms:**
```
Database connection failed: timeout
```

**Solution:**
1. Verify `DATABASE_URL` format:
   ```bash
   postgres://user:password@ep-your-endpoint.neon.tech/dbname?sslmode=require
   ```

2. Check Neon database is active (not suspended)

3. Test connection from Railway:
   ```bash
   # In Railway terminal
   psql $DATABASE_URL -c "SELECT 1"
   ```

4. Verify firewall rules allow Railway to connect

---

### Issue: Vector Store Not Found

**Symptoms:**
```json
{
  "vector_store": "degraded",
  "details": {
    "vector_store": "Collection 'robotics_textbook' not found"
  }
}
```

**Solution:**
1. Verify collection exists in Qdrant dashboard

2. Check collection name matches:
   ```bash
   QDRANT_COLLECTION=robotics_textbook
   ```

3. If collection is missing, re-index content:
   ```bash
   # Run content indexing script
   python backend/scripts/index_content.py
   ```

---

## Monitoring and Logs

### View Railway Logs

```bash
# Using Railway CLI
railway logs

# Filter by keyword
railway logs | grep "ERROR"
railway logs | grep "Query"
```

### Key Metrics to Monitor

1. **Response Times**
   - Health checks: < 1s
   - Queries: < 5s (non-streaming)
   - Streaming: First chunk < 2s

2. **Error Rates**
   - Target: < 1% error rate
   - Check logs for patterns

3. **Service Health**
   - Database: Should always be healthy
   - LLM: Should always be healthy
   - Vector store: Degraded is acceptable if collection exists

4. **Rate Limiting**
   - Monitor rate limit headers
   - Adjust limits based on usage

---

## Success Criteria

Your deployment is successful if:

- [ ] All health endpoints return 200 OK
- [ ] Detailed health shows all services as "healthy"
- [ ] CORS headers are present in responses
- [ ] Query endpoint returns valid answers with sources
- [ ] Streaming endpoint delivers chunks progressively
- [ ] Frontend can make requests without CORS errors
- [ ] Session management endpoints work correctly
- [ ] Rate limiting headers are present
- [ ] Response times are acceptable
- [ ] No errors in Railway logs

---

## Quick Commands Reference

```bash
# Set your Railway URL
export RAILWAY_URL="https://your-app.up.railway.app"

# Quick health check
curl $RAILWAY_URL/health

# Detailed health
curl $RAILWAY_URL/api/v1/health | jq '.'

# Test query
curl -X POST $RAILWAY_URL/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS?", "filters": {}, "top_k": 3}' | jq '.'

# Test CORS
curl -I -X OPTIONS $RAILWAY_URL/api/v1/health \
  -H "Origin: https://sajid-khan-afridi.github.io"

# Run full test suite
python test_railway_deployment.py $RAILWAY_URL

# Quick bash test
./test_railway_deployment.sh $RAILWAY_URL
```

---

## Need Help?

- Check Railway logs: `railway logs`
- Review environment variables in Railway dashboard
- Verify external service status (Neon, Qdrant, OpenAI)
- Test each service independently using health endpoints
- Use the browser test tool for real CORS testing

---

**Last Updated**: 2025-12-17
