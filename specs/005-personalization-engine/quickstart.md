# Quickstart Guide: Phase 4B Personalization Engine

**Feature**: Phase 4B Personalization Engine
**Branch**: `1-personalization-engine`
**Date**: 2025-12-22
**Prerequisites**: Phase 4A authentication must be functional (Better Auth, ProfileService, user_profiles table)

## Overview

This guide walks you through setting up the Phase 4B Personalization Engine on your local development environment, including database schema, backend services, frontend components, and testing.

---

## Prerequisites

### Required Software
- **Python**: 3.11+ (backend)
- **Node.js**: 18+ and npm (frontend)
- **PostgreSQL**: Neon Serverless account (free tier)
- **Git**: For version control

### Required Credentials
- **Neon Database URL**: `postgresql://[user]:[password]@[host]/[database]`
- **OpenAI API Key**: For profile-aware RAG chatbot (from Phase 3)
- **Better Auth Secrets**: From Phase 4A setup

### Phase 4A Verification
Before proceeding, ensure Phase 4A is functional:
```bash
# Test authentication endpoint
curl http://localhost:8000/api/v1/auth/session

# Test profile retrieval
curl -H "Authorization: Bearer YOUR_JWT_TOKEN" \
     http://localhost:8000/api/v1/profile
```

If Phase 4A endpoints fail, complete Phase 4A setup first.

---

## Step 1: Environment Setup

### 1.1 Clone Repository (if not already done)
```bash
git clone https://github.com/your-org/robotics-textbook.git
cd robotics-textbook
git checkout 1-personalization-engine
```

### 1.2 Backend Environment Configuration
Create or update `backend/.env`:
```bash
# Neon Postgres Database
DATABASE_URL=postgresql://[user]:[password]@[host]/[database]?sslmode=require

# Better Auth (from Phase 4A)
BETTER_AUTH_SECRET=your-secret-key-from-phase-4a
BETTER_AUTH_URL=http://localhost:8000

# OpenAI API (from Phase 3)
OPENAI_API_KEY=sk-...

# Personalization Settings (new for Phase 4B)
RECOMMENDATION_CACHE_TTL=3600           # 1 hour in seconds
RECOMMENDATION_CACHE_MAX_SIZE=1000      # Max 1000 users cached
CLASSIFICATION_ALGORITHM_VERSION=1.0    # For tracking algorithm changes

# Rate Limiting (new for Phase 4B)
PROGRESS_RATE_LIMIT=100                 # 100 requests/hour per user
RECOMMENDATION_RATE_LIMIT=50            # 50 requests/hour per user
```

### 1.3 Frontend Environment Configuration
Create or update `frontend/.env.local`:
```bash
# Backend API URL
NEXT_PUBLIC_API_URL=http://localhost:8000/api/v1

# Feature Flags (new for Phase 4B)
NEXT_PUBLIC_ENABLE_RECOMMENDATIONS=true
NEXT_PUBLIC_ENABLE_PROGRESS_TRACKING=true
NEXT_PUBLIC_ENABLE_ADAPTIVE_CONTENT=true
```

---

## Step 2: Database Schema Migration

### 2.1 Connect to Neon Database
```bash
# Install PostgreSQL client tools if not already installed
# For macOS:
brew install postgresql

# For Ubuntu/Debian:
sudo apt-get install postgresql-client

# Test connection
psql "$DATABASE_URL" -c "SELECT version();"
```

### 2.2 Run Phase 4B Migration
```bash
cd backend

# Apply migration script
psql "$DATABASE_URL" < src/migrations/005_personalization_schema.sql

# Verify tables created
psql "$DATABASE_URL" -c "\dt"

# Expected output should include:
# - skill_level_classifications
# - chapter_progress
# - chapter_metadata
```

### 2.3 Verify Chapter Metadata Populated
```bash
# Check chapter metadata loaded
psql "$DATABASE_URL" -c "SELECT chapter_id, title, difficulty_level FROM chapter_metadata LIMIT 5;"

# Expected: 10 rows with module-1, module-2, module-3 chapters
```

---

## Step 3: Backend Installation

### 3.1 Install Python Dependencies
```bash
cd backend

# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Install new Phase 4B dependencies
pip install cachetools  # For recommendation caching
pip install asyncpg     # Async PostgreSQL driver (if not already installed)
```

### 3.2 Verify Dependency Installation
```bash
# Test imports
python -c "import cachetools; print('cachetools:', cachetools.__version__)"
python -c "import asyncpg; print('asyncpg:', asyncpg.__version__)"
```

---

## Step 4: Backend Services Implementation

### 4.1 File Structure Check
Verify these new files exist (created during implementation):
```bash
tree backend/src/

# Expected new files:
# backend/src/models/skill_level_classification.py
# backend/src/models/chapter_progress.py
# backend/src/models/chapter_recommendation.py
# backend/src/models/chapter_metadata.py
# backend/src/services/classification_service.py
# backend/src/services/progress_tracking_service.py
# backend/src/services/recommendation_service.py
# backend/src/services/personalization_service.py
# backend/src/api/routes/personalization.py
# backend/src/api/routes/progress.py
```

### 4.2 Run Backend Server
```bash
cd backend
source venv/bin/activate

# Start FastAPI server with hot reload
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000

# Server should start at http://localhost:8000
# OpenAPI docs available at http://localhost:8000/docs
```

### 4.3 Test Backend Endpoints
```bash
# In a new terminal window:

# 1. Test skill level classification (requires authentication)
curl -H "Authorization: Bearer YOUR_JWT_TOKEN" \
     http://localhost:8000/api/v1/skill-level

# Expected: {"user_id": "...", "skill_level": "intermediate", ...}

# 2. Test chapter progress tracking
curl -X POST \
     -H "Authorization: Bearer YOUR_JWT_TOKEN" \
     -H "Content-Type: application/json" \
     -d '{"chapter_id": "module-1/ros-intro"}' \
     http://localhost:8000/api/v1/progress/start

# Expected: {"id": "...", "chapter_id": "module-1/ros-intro", "status": "started", ...}

# 3. Test recommendations
curl -H "Authorization: Bearer YOUR_JWT_TOKEN" \
     http://localhost:8000/api/v1/recommendations

# Expected: {"recommendations": [...], "cached": false}
```

---

## Step 5: Frontend Installation

### 5.1 Install Node Dependencies
```bash
cd frontend

# Install dependencies
npm install

# Install new Phase 4B dependencies (if not in package.json yet)
npm install @tanstack/react-query  # Optional: for data fetching/caching
```

### 5.2 Verify Frontend File Structure
```bash
tree frontend/src/

# Expected new files:
# frontend/src/components/PersonalizedSection.tsx (ENHANCED)
# frontend/src/components/RecommendationCard.tsx
# frontend/src/components/ProgressTracker.tsx
# frontend/src/hooks/useSkillLevel.ts
# frontend/src/hooks/useChapterProgress.ts
# frontend/src/hooks/useRecommendations.ts
# frontend/src/services/personalization-api.ts
```

### 5.3 Run Frontend Development Server
```bash
cd frontend

# Start Docusaurus dev server
npm run start

# Server should start at http://localhost:3000
```

### 5.4 Test Frontend Features
1. **Navigate to chapter page** (e.g., http://localhost:3000/docs/module-1/ros-intro)
2. **Wait 10 seconds** → Check Network tab for `/api/v1/progress/start` call
3. **Scroll to bottom** → Check Network tab for `/api/v1/progress/complete` call
4. **Click bookmark icon** → Check Network tab for `/api/v1/progress/bookmark` call
5. **Visit dashboard** → Should see recommendations displayed

---

## Step 6: Testing

### 6.1 Backend Unit Tests
```bash
cd backend
source venv/bin/activate

# Run all tests
pytest tests/ -v

# Run specific test suites
pytest tests/unit/test_classification_service.py -v
pytest tests/unit/test_progress_service.py -v
pytest tests/unit/test_recommendation_service.py -v

# Run with coverage report
pytest tests/ --cov=src --cov-report=html

# Expected: 80%+ coverage
```

### 6.2 Backend Integration Tests
```bash
cd backend

# Run integration tests (requires test database)
pytest tests/integration/test_personalization_api.py -v

# Expected: All API endpoints return correct responses
```

### 6.3 Frontend Component Tests
```bash
cd frontend

# Run all tests
npm test

# Run specific test suites
npm test -- --testPathPattern=PersonalizedSection
npm test -- --testPathPattern=RecommendationCard
npm test -- --testPathPattern=ProgressTracker

# Run with coverage
npm test -- --coverage

# Expected: 80%+ coverage for new components
```

### 6.4 End-to-End Tests
```bash
cd frontend

# Run Playwright E2E tests
npx playwright test tests/e2e/personalization.spec.ts

# Run with UI (for debugging)
npx playwright test --ui

# Expected: All critical user journeys pass
# - Skill level classification on profile completion
# - Progress tracking on chapter view
# - Recommendations display on dashboard
```

---

## Step 7: Verify Phase 4B Quality Gates

### 7.1 Skill Level Classification Functional
```bash
# Test classification consistency
curl -H "Authorization: Bearer USER1_TOKEN" http://localhost:8000/api/v1/skill-level
curl -H "Authorization: Bearer USER1_TOKEN" http://localhost:8000/api/v1/skill-level

# Expected: Same skill_level returned both times (deterministic)
```

### 7.2 Personalized Recommendations Display Correctly
```bash
# Test recommendations endpoint
curl -H "Authorization: Bearer USER1_TOKEN" http://localhost:8000/api/v1/recommendations

# Expected: Array of 1-3 recommendations with relevance_score > 0.0
```

### 7.3 Response Time < 2s
```bash
# Measure personalization response time
time curl -H "Authorization: Bearer USER1_TOKEN" http://localhost:8000/api/v1/recommendations

# Expected: Total time < 2 seconds (first call), < 0.1s (cached calls)
```

### 7.4 Recommendation Relevance > 0.75
```bash
# Check average relevance score
curl -H "Authorization: Bearer USER1_TOKEN" http://localhost:8000/api/v1/recommendations | \
jq '.recommendations | map(.relevance_score) | add / length'

# Expected: Average score > 0.75
```

---

## Step 8: Common Issues & Troubleshooting

### Issue 1: Database Connection Fails
**Symptom**: `could not connect to server: Connection refused`

**Solution**:
```bash
# Verify DATABASE_URL is correct
echo $DATABASE_URL

# Test connection directly
psql "$DATABASE_URL" -c "SELECT 1;"

# Check Neon dashboard for database status
```

### Issue 2: Skill Classification Returns Default "intermediate" for All Users
**Symptom**: All users classified as "intermediate" regardless of profile

**Solution**:
```bash
# Check user profiles exist
psql "$DATABASE_URL" -c "SELECT user_id, experience_level, ros_familiarity FROM user_profiles LIMIT 5;"

# Verify classification algorithm is applied
# Check backend logs for classification computation
grep "skill_score" backend/logs/app.log
```

### Issue 3: Recommendations API Returns Empty Array
**Symptom**: `{"recommendations": [], "cached": false}`

**Solution**:
```bash
# Check chapter_metadata table populated
psql "$DATABASE_URL" -c "SELECT COUNT(*) FROM chapter_metadata;"
# Expected: 10 rows

# Check user has not completed all chapters
psql "$DATABASE_URL" -c "SELECT COUNT(*) FROM chapter_progress WHERE user_id='USER_ID' AND status='completed';"
# Expected: < 10

# Check prerequisites not blocking all chapters
# Review recommendation algorithm logs
```

### Issue 4: Progress Tracking Not Triggering
**Symptom**: Frontend timer reaches 10s but no API call made

**Solution**:
```bash
# Check authentication token is valid
curl -H "Authorization: Bearer YOUR_JWT_TOKEN" http://localhost:8000/api/v1/auth/session

# Check browser console for errors
# Verify useChapterProgress hook is invoked on chapter pages

# Check rate limiting not blocking requests
curl -i -H "Authorization: Bearer YOUR_JWT_TOKEN" \
     -X POST -d '{"chapter_id": "module-1/ros-intro"}' \
     http://localhost:8000/api/v1/progress/start
# Look for HTTP 429 (rate limit exceeded)
```

### Issue 5: Recommendation Cache Not Invalidating
**Symptom**: Stale recommendations persist after profile update

**Solution**:
```bash
# Check cache invalidation logic in code
# ProfileService.update_profile() should call invalidate_recommendation_cache(user_id)

# Verify cache invalidation logs
grep "cache_invalidated" backend/logs/app.log

# Force refresh via API
curl -H "Authorization: Bearer YOUR_JWT_TOKEN" \
     "http://localhost:8000/api/v1/recommendations?force_refresh=true"
```

---

## Step 9: Production Deployment Checklist

Before deploying to Railway/Render:

### 9.1 Environment Variables
- [ ] `DATABASE_URL` set to production Neon database
- [ ] `OPENAI_API_KEY` set with production key
- [ ] `BETTER_AUTH_SECRET` set with secure random string
- [ ] `BETTER_AUTH_URL` set to production backend URL
- [ ] Rate limiting values appropriate for production load

### 9.2 Database
- [ ] Run migration on production database
- [ ] Verify chapter_metadata populated (10 chapters)
- [ ] Test database connection from production server

### 9.3 Backend
- [ ] All unit tests passing (80%+ coverage)
- [ ] All integration tests passing
- [ ] No secrets in code (use environment variables)
- [ ] Structured JSON logging enabled
- [ ] Error handling and correlation IDs implemented

### 9.4 Frontend
- [ ] All component tests passing
- [ ] E2E tests passing (Playwright)
- [ ] Build succeeds: `npm run build`
- [ ] Environment variables set for production API URL

### 9.5 Performance
- [ ] Skill classification < 500ms (measured)
- [ ] Personalization response < 2s (measured)
- [ ] Recommendation cache hit rate > 70% (monitored)

### 9.6 Security
- [ ] JWT tokens validated on all authenticated endpoints
- [ ] Input sanitization implemented (chapter_id, user_id)
- [ ] Rate limiting functional (tested with load tool)
- [ ] No PII in logs (verified)

---

## Step 10: Monitoring & Observability

### 10.1 Backend Metrics to Track
```python
# Add to backend monitoring dashboard:
# - Skill classification latency (p50, p95, p99)
# - Recommendation API latency (p50, p95, p99)
# - Progress tracking API latency (p50, p95, p99)
# - Recommendation cache hit rate (hits / total requests)
# - Average recommendation relevance score
# - Error rates per endpoint
```

### 10.2 Frontend Metrics to Track
```typescript
// Add to frontend analytics:
// - Time to first recommendation displayed
// - Recommendation click-through rate
// - Progress tracking event success rate
// - Personalization feature adoption (% users with progress records)
```

### 10.3 Database Monitoring
```bash
# Monitor Neon dashboard for:
# - Storage usage (should stay well under 0.5GB)
# - Connection pool usage (max 10 connections)
# - Query performance (slow queries > 1s)
```

---

## Next Steps

After completing this quickstart:

1. **Generate tasks.md**: Run `/sp.tasks` to break down implementation into testable tasks
2. **Implement backend services**: Start with ClassificationService (P1)
3. **Implement frontend components**: Enhance PersonalizedSection (P2)
4. **Write tests**: Achieve 80%+ coverage before marking tasks complete
5. **Deploy to staging**: Test in Railway/Render staging environment
6. **User acceptance testing**: Validate with real users
7. **Deploy to production**: After all quality gates pass

---

## Additional Resources

- **OpenAPI Docs**: http://localhost:8000/docs (when backend running)
- **Database Schema**: See `data-model.md`
- **API Contracts**: See `contracts/personalization-api.yaml`
- **Research Decisions**: See `research.md`
- **Constitution**: See `.specify/memory/constitution.md`

---

## Support

If you encounter issues not covered in this guide:
1. Check backend logs: `backend/logs/app.log`
2. Check frontend console: Browser DevTools → Console
3. Review Neon database logs: Neon dashboard → Logs
4. Create GitHub issue with:
   - Error message + stack trace
   - Steps to reproduce
   - Environment details (OS, Python/Node version)
   - Relevant logs (redact secrets)

---

**Quickstart Complete!** Your Phase 4B Personalization Engine should now be running locally. Proceed to implementation via `/sp.tasks` command.
