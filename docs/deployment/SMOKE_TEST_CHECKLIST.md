# Smoke Test Checklist - Phase 4B Staging

**Purpose**: Validate Phase 4B Personalization Engine deployment to staging
**Execution Time**: 15-20 minutes
**Prerequisites**: Staging backend and frontend deployed
**Date**: 2025-12-23

---

## Automated Smoke Test

### Run Automated Script

```bash
# Navigate to project
cd "D:\GitHub Connected\hackathon1_repeat"

# Run smoke test script
./backend/scripts/smoke_test_staging.sh https://hackathon1-staging.up.railway.app

# Review results
cat smoke_test_results_*.log
```

**Expected Output**:
```
✅ All smoke tests PASSED
Ready for UAT
```

**Pass Criteria**: Pass rate ≥ 80%

---

## Manual Smoke Test Checklist

Execute the following tests manually to validate critical functionality.

### Section 1: Infrastructure Health

- [ ] **Test 1.1: Backend Health Check**
  - Navigate to: `https://hackathon1-staging.up.railway.app/health`
  - **Expected**: `{"status":"ok"}`
  - **Result**: ______________________

- [ ] **Test 1.2: Detailed Health Check**
  - Navigate to: `https://hackathon1-staging.up.railway.app/api/v1/health`
  - **Expected**: JSON with database, Qdrant, OpenAI status all "ok"
  - **Result**: ______________________

- [ ] **Test 1.3: API Documentation**
  - Navigate to: `https://hackathon1-staging.up.railway.app/api/docs`
  - **Expected**: Swagger UI loads with all Phase 4B endpoints visible
  - **Result**: ______________________

- [ ] **Test 1.4: Frontend Loads**
  - Navigate to: `https://hackathon1-staging.vercel.app` (or your staging URL)
  - **Expected**: Homepage loads without errors
  - **Result**: ______________________

- [ ] **Test 1.5: No Console Errors**
  - Open browser DevTools → Console
  - **Expected**: No red errors (warnings acceptable)
  - **Result**: ______________________

---

### Section 2: Database Connectivity

- [ ] **Test 2.1: Database Tables Exist**
  ```bash
  psql "$STAGING_DB" -c "
  SELECT table_name FROM information_schema.tables
  WHERE table_schema = 'public'
    AND table_name IN ('skill_level_classifications', 'chapter_progress', 'chapter_metadata')
  ORDER BY table_name;
  "
  ```
  - **Expected**: 3 rows returned
  - **Result**: ______________________

- [ ] **Test 2.2: Chapter Metadata Seeded**
  ```bash
  psql "$STAGING_DB" -c "SELECT COUNT(*) FROM chapter_metadata;"
  ```
  - **Expected**: 10 rows
  - **Result**: ______________________

- [ ] **Test 2.3: Foreign Keys Enforced**
  ```bash
  # Try inserting invalid user_id (should fail)
  psql "$STAGING_DB" -c "
  INSERT INTO skill_level_classifications (user_id, skill_level, based_on_profile)
  VALUES ('00000000-0000-0000-0000-000000000000', 'beginner', '{}'::jsonb);
  "
  ```
  - **Expected**: Error: violates foreign key constraint
  - **Result**: ______________________

---

### Section 3: Authentication & User Management

- [ ] **Test 3.1: Google OAuth Flow**
  - Navigate to frontend staging
  - Click "Login with Google"
  - Complete Google OAuth flow
  - **Expected**: Redirected back to frontend, logged in
  - **Result**: ______________________

- [ ] **Test 3.2: GitHub OAuth Flow**
  - Navigate to frontend staging
  - Click "Login with GitHub"
  - Complete GitHub OAuth flow
  - **Expected**: Redirected back to frontend, logged in
  - **Result**: ______________________

- [ ] **Test 3.3: User Profile Created**
  - After login, navigate to `/profile`
  - **Expected**: User profile page loads with user info
  - **Result**: ______________________

- [ ] **Test 3.4: Session Persistence**
  - Refresh page after login
  - **Expected**: Still logged in (session persists)
  - **Result**: ______________________

---

### Section 4: Phase 4B Skill Classification

- [ ] **Test 4.1: Skill Level Endpoint (Unauthenticated)**
  ```bash
  curl https://hackathon1-staging.up.railway.app/api/v1/skill-level
  ```
  - **Expected**: HTTP 401 Unauthorized
  - **Result**: ______________________

- [ ] **Test 4.2: Create User Profile**
  - Navigate to `/profile` (while logged in)
  - Fill out profile form:
    - ROS Experience: None
    - Python Proficiency: Beginner
    - Hardware Access: No
    - Learning Goals: Theoretical, Practical
  - Click "Save"
  - **Expected**: Profile saved successfully
  - **Result**: ______________________

- [ ] **Test 4.3: Skill Level Calculated**
  - After saving profile, check skill level badge
  - **Expected**: Badge displays "Beginner" (based on profile)
  - **Result**: ______________________

- [ ] **Test 4.4: Skill Level API Response**
  ```bash
  # Get auth token from browser (Application → Cookies → access_token)
  export TOKEN="your-access-token"

  curl -H "Authorization: Bearer $TOKEN" \
    https://hackathon1-staging.up.railway.app/api/v1/skill-level
  ```
  - **Expected**: JSON with `{"skill_level":"beginner",...}`
  - **Result**: ______________________

- [ ] **Test 4.5: Skill Level Recalculation**
  - Update profile: Change Python Proficiency to "Advanced"
  - Save profile
  - **Expected**: Skill level badge updates to "Intermediate" or "Advanced"
  - **Result**: ______________________

---

### Section 5: Chapter Recommendations

- [ ] **Test 5.1: Recommendations Endpoint (Authenticated)**
  ```bash
  curl -H "Authorization: Bearer $TOKEN" \
    https://hackathon1-staging.up.railway.app/api/v1/recommendations
  ```
  - **Expected**: JSON array with recommended chapters
  - **Result**: ______________________

- [ ] **Test 5.2: Recommendations Display on Homepage**
  - Navigate to homepage (while logged in)
  - **Expected**: "Recommended for You" section displays
  - **Expected**: Chapters appropriate for skill level shown
  - **Result**: ______________________

- [ ] **Test 5.3: Recommendations Match Skill Level**
  - For Beginner user: Expect Module 1 chapters (beginner difficulty)
  - For Intermediate: Expect Module 2 chapters
  - For Advanced: Expect Module 3 chapters
  - **Expected**: Recommendations align with user skill level
  - **Result**: ______________________

- [ ] **Test 5.4: Hardware Filter Applied**
  - If user has no hardware access
  - **Expected**: No chapters with `requires_hardware=true` in recommendations
  - **Result**: ______________________

---

### Section 6: Progress Tracking

- [ ] **Test 6.1: Start Chapter**
  - From recommendations, click "Start Chapter" on a beginner chapter
  - **Expected**: Chapter page loads, progress marked as "started"
  - **Result**: ______________________

- [ ] **Test 6.2: Progress Saved to Database**
  ```bash
  # Check database for progress record
  psql "$STAGING_DB" -c "
  SELECT chapter_id, status, started_at
  FROM chapter_progress
  WHERE user_id = (SELECT id FROM users WHERE email = 'your-test-email@gmail.com')
  LIMIT 5;
  "
  ```
  - **Expected**: Row with chapter_id and status='started'
  - **Result**: ______________________

- [ ] **Test 6.3: Complete Chapter**
  - On chapter page, click "Mark as Complete"
  - **Expected**: Status updates to "completed", completed_at timestamp set
  - **Result**: ______________________

- [ ] **Test 6.4: Bookmark Chapter**
  - On chapter page, click "Bookmark"
  - **Expected**: Bookmark icon changes, is_bookmarked=true in database
  - **Result**: ______________________

- [ ] **Test 6.5: Progress Displays on Profile**
  - Navigate to `/profile`
  - **Expected**: "Your Progress" section shows started/completed chapters
  - **Result**: ______________________

- [ ] **Test 6.6: Progress API Endpoint**
  ```bash
  curl -X POST -H "Authorization: Bearer $TOKEN" \
    -H "Content-Type: application/json" \
    -d '{"chapter_id":"module-1/ros-intro"}' \
    https://hackathon1-staging.up.railway.app/api/v1/progress/start
  ```
  - **Expected**: HTTP 200, JSON with progress record
  - **Result**: ______________________

---

### Section 7: RAG Chatbot with Personalization

- [ ] **Test 7.1: Chatbot Page Loads**
  - Navigate to `/chatbot`
  - **Expected**: Chatbot UI loads, no errors
  - **Result**: ______________________

- [ ] **Test 7.2: Ask Question (Beginner User)**
  - Login as beginner user
  - Ask: "What is ROS 2?"
  - **Expected**: Response simplified for beginner level
  - **Expected**: Response mentions "new to ROS" or similar beginner context
  - **Result**: ______________________

- [ ] **Test 7.3: Ask Question (Advanced User)**
  - Login as advanced user
  - Ask: "Explain ROS 2 DDS middleware"
  - **Expected**: Response includes technical details appropriate for advanced users
  - **Result**: ______________________

- [ ] **Test 7.4: Context Includes Progress**
  - Start chapter "module-1/ros-intro"
  - Ask chatbot: "What should I learn next?"
  - **Expected**: Response acknowledges started chapter, suggests next steps
  - **Result**: ______________________

- [ ] **Test 7.5: Chat History Persists**
  - Ask a question
  - Refresh page
  - **Expected**: Previous conversation visible in chat history
  - **Result**: ______________________

---

### Section 8: CORS & Security

- [ ] **Test 8.1: CORS Allows Staging Frontend**
  - On frontend staging, open Network tab
  - Make any API call (e.g., fetch recommendations)
  - **Expected**: No CORS errors in console
  - **Result**: ______________________

- [ ] **Test 8.2: CORS Blocks Unauthorized Origins**
  - On different domain (e.g., https://example.com), open console:
  ```javascript
  fetch('https://hackathon1-staging.up.railway.app/health')
    .then(r => r.json())
    .then(console.log);
  ```
  - **Expected**: CORS error (blocked by policy)
  - **Result**: ______________________

- [ ] **Test 8.3: HTTPS Enforced**
  - Try accessing: `http://hackathon1-staging.up.railway.app/health`
  - **Expected**: Redirects to HTTPS or connection refused
  - **Result**: ______________________

- [ ] **Test 8.4: JWT Token Required for Protected Endpoints**
  ```bash
  # Try accessing recommendations without token
  curl https://hackathon1-staging.up.railway.app/api/v1/recommendations
  ```
  - **Expected**: HTTP 401 Unauthorized
  - **Result**: ______________________

---

### Section 9: Performance

- [ ] **Test 9.1: Homepage Load Time**
  - Use browser DevTools → Network tab
  - Load homepage
  - **Expected**: DOMContentLoaded < 2 seconds
  - **Result**: ______________________

- [ ] **Test 9.2: API Response Time**
  ```bash
  time curl https://hackathon1-staging.up.railway.app/health
  ```
  - **Expected**: < 200ms
  - **Result**: ______________________

- [ ] **Test 9.3: Recommendations Response Time**
  ```bash
  time curl -H "Authorization: Bearer $TOKEN" \
    https://hackathon1-staging.up.railway.app/api/v1/recommendations
  ```
  - **Expected**: < 1000ms
  - **Result**: ______________________

- [ ] **Test 9.4: Database Query Performance**
  ```bash
  psql "$STAGING_DB" -c "EXPLAIN ANALYZE SELECT * FROM chapter_progress LIMIT 10;"
  ```
  - **Expected**: Execution time < 50ms
  - **Result**: ______________________

---

### Section 10: Error Handling

- [ ] **Test 10.1: Invalid Chapter ID**
  ```bash
  curl -X POST -H "Authorization: Bearer $TOKEN" \
    -H "Content-Type: application/json" \
    -d '{"chapter_id":"invalid-chapter-id"}' \
    https://hackathon1-staging.up.railway.app/api/v1/progress/start
  ```
  - **Expected**: HTTP 422 or 404 with error message
  - **Result**: ______________________

- [ ] **Test 10.2: Missing Required Fields**
  ```bash
  curl -X POST -H "Authorization: Bearer $TOKEN" \
    -H "Content-Type: application/json" \
    -d '{}' \
    https://hackathon1-staging.up.railway.app/api/v1/progress/start
  ```
  - **Expected**: HTTP 422 with validation error
  - **Result**: ______________________

- [ ] **Test 10.3: Invalid Skill Level Enum**
  ```bash
  # Try inserting invalid skill level directly in DB
  psql "$STAGING_DB" -c "
  INSERT INTO skill_level_classifications (user_id, skill_level, based_on_profile)
  SELECT id, 'expert', '{}'::jsonb FROM users LIMIT 1;
  "
  ```
  - **Expected**: Error: check constraint violation
  - **Result**: ______________________

---

## Smoke Test Results Summary

**Test Date**: ______________________
**Tested By**: ______________________
**Environment**: Staging

### Results

| Section | Total Tests | Passed | Failed | Pass Rate |
|---------|-------------|--------|--------|-----------|
| 1. Infrastructure | 5 | ___ | ___ | ___% |
| 2. Database | 3 | ___ | ___ | ___% |
| 3. Authentication | 4 | ___ | ___ | ___% |
| 4. Skill Classification | 5 | ___ | ___ | ___% |
| 5. Recommendations | 4 | ___ | ___ | ___% |
| 6. Progress Tracking | 6 | ___ | ___ | ___% |
| 7. RAG Chatbot | 5 | ___ | ___ | ___% |
| 8. CORS & Security | 4 | ___ | ___ | ___% |
| 9. Performance | 4 | ___ | ___ | ___% |
| 10. Error Handling | 3 | ___ | ___ | ___% |
| **TOTAL** | **43** | ___ | ___ | ___% |

### Overall Assessment

- [ ] **PASS**: All critical tests passed (pass rate ≥ 90%)
- [ ] **PASS WITH WARNINGS**: Most tests passed (pass rate ≥ 80%)
- [ ] **FAIL**: Critical failures found (pass rate < 80%)

### Critical Issues Found

(List any blocking issues that prevent UAT)

1. ______________________
2. ______________________
3. ______________________

### Non-Critical Issues

(List minor issues that can be addressed during UAT)

1. ______________________
2. ______________________

### Sign-off

**Smoke Test Outcome**: ☐ PASS  ☐ FAIL

**Ready for UAT**: ☐ YES  ☐ NO (fix critical issues first)

**Tester Signature**: ______________________

**Date**: ______________________

---

## Next Steps

- ✅ **If PASS**: Proceed to UAT (see `UAT_TEST_PLAN.md`)
- ❌ **If FAIL**: Fix critical issues, redeploy, rerun smoke tests

---

**Document Version**: 1.0
**Last Updated**: 2025-12-23
**Maintained By**: QA Team
