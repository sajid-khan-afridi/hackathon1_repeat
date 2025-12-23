# Phase 4B Smoke Test Checklist

**Purpose**: Manual smoke test checklist for staging deployment validation
**Time Required**: 15-20 minutes
**Prerequisites**: Staging deployment complete

---

## Environment Information

- **Staging Frontend**: https://staging-hackathon1.vercel.app
- **Staging Backend**: https://hackathon1-staging.up.railway.app
- **API Docs**: https://hackathon1-staging.up.railway.app/docs
- **Tester**: [Your Name]
- **Date**: [Date]
- **Browser**: [Browser + Version]

---

## Test Checklist

### 1. Infrastructure Health

- [ ] **Frontend loads successfully**
  - Navigate to staging frontend URL
  - No 404 or 500 errors
  - Page renders correctly

- [ ] **Backend health check passes**
  - Navigate to: https://hackathon1-staging.up.railway.app/health
  - Response: `{"status": "healthy", "environment": "staging"}`

- [ ] **API documentation accessible**
  - Navigate to: https://hackathon1-staging.up.railway.app/docs
  - FastAPI docs page loads
  - Phase 4B endpoints visible: `/api/v1/skill-level`, `/api/v1/recommendations`, `/api/v1/progress/*`

---

### 2. Authentication & Profile Setup

- [ ] **Google OAuth login works**
  - Click "Sign In" → "Continue with Google"
  - Successfully authenticates
  - Redirected to profile setup or dashboard

- [ ] **Profile settings page loads**
  - Navigate to profile settings
  - All fields visible:
    - Programming Experience (dropdown)
    - ROS Familiarity (dropdown)
    - Preferred Language (radio buttons)
    - Hardware Access (radio buttons)
    - Learning Goal (dropdown)

- [ ] **Profile can be saved**
  - Fill in all profile fields
  - Click "Save Profile"
  - Success message displayed
  - Profile persists after page refresh

---

### 3. Skill Level Classification (T011-T012)

- [ ] **Skill level calculated on profile save**
  - After saving profile, skill level badge appears
  - Badge shows: Beginner / Intermediate / Advanced
  - Badge matches expected level based on profile

- [ ] **Skill level API endpoint works**
  - Open browser DevTools → Network tab
  - Refresh page
  - Check API call: `GET /api/v1/skill-level`
  - Response status: 200 OK
  - Response includes: `{"skill_level": "...", "calculated_at": "..."}`

- [ ] **Skill level updates when profile changes**
  - Change profile (e.g., Beginner → Advanced)
  - Save profile
  - Skill level badge updates accordingly

---

### 4. Chapter Recommendations (T031-T033)

- [ ] **Recommendations displayed on dashboard**
  - Navigate to dashboard/home page
  - "Recommended for You" section visible
  - 3 chapter recommendations shown

- [ ] **Each recommendation has required info**
  - Chapter title visible
  - Relevance score or indicator shown
  - Reason for recommendation displayed
  - "Start Learning" or chapter link present

- [ ] **Recommendations match user profile**
  - Beginner user → See Module 1 beginner chapters
  - Advanced user → See Module 3 advanced chapters
  - Recommendations align with learning goal

- [ ] **Recommendations API works**
  - DevTools → Network tab
  - Check API call: `GET /api/v1/recommendations`
  - Response status: 200 OK
  - Response includes: `{"recommendations": [...]}`

---

### 5. Progress Tracking (T021-T025)

- [ ] **Chapter auto-starts after 10 seconds**
  - Navigate to any chapter
  - Wait 10 seconds
  - Check progress indicator → Shows "Started" or "In Progress"

- [ ] **Chapter completion detection works**
  - Scroll to bottom of chapter
  - Progress indicator updates → Shows "Completed" ✓
  - Completion timestamp recorded

- [ ] **Bookmark functionality works**
  - Click bookmark icon on chapter
  - Icon toggles to "bookmarked" state
  - Click again → Unbookmarks

- [ ] **Progress persists across sessions**
  - Complete a chapter
  - Sign out
  - Sign back in
  - Navigate to progress/dashboard → Completed chapter still marked

- [ ] **Progress API endpoints work**
  - DevTools → Network tab
  - Check API calls:
    - `POST /api/v1/progress/start` (when chapter starts)
    - `POST /api/v1/progress/complete` (when chapter completes)
    - `GET /api/v1/progress` (when loading progress summary)
  - All return 200 OK

---

### 6. Personalized Content (T060-T062)

- [ ] **Language filtering works**
  - Sign in as Python user
  - Navigate to a chapter with code examples
  - Python code examples visible
  - C++ code examples hidden or collapsed
  - Sign out, sign in as C++ user
  - C++ code examples visible
  - Python code examples hidden or collapsed

- [ ] **Experience level filtering works**
  - Sign in as beginner user
  - Navigate to chapter
  - Beginner explanations visible
  - Advanced sections hidden or collapsed
  - Sign in as advanced user
  - Advanced sections visible

- [ ] **Hardware filtering works**
  - Sign in as "Simulation Only" user
  - Hardware setup instructions hidden
  - Sign in as "Physical Robot" user
  - Hardware setup instructions visible

---

### 7. Profile-Aware RAG Chatbot (T071-T072)

- [ ] **Chatbot loads successfully**
  - Open chatbot widget/page
  - Chatbot interface displays
  - Input field and send button visible

- [ ] **Chatbot provides personalized responses**
  - **Test as Beginner Python user**:
    - Ask: "How do I create a ROS publisher?"
    - Response includes:
      - Python code example (not C++)
      - Beginner-friendly explanation
      - No advanced terminology assumed

  - **Test as Advanced C++ user**:
    - Ask same question
    - Response includes:
      - C++ code example (not Python)
      - Advanced details (QoS, lifecycle, etc.)

- [ ] **Chatbot response time acceptable**
  - Ask a question
  - Response appears in < 3 seconds

- [ ] **Context is preserved in conversation**
  - Ask: "What is inverse kinematics?"
  - Follow up: "Show me a code example"
  - Second response understands context (IK code example)

---

### 8. Performance Requirements

- [ ] **Skill classification < 500ms**
  - Save profile
  - Check Network tab timing for `/api/v1/skill-level`
  - Time < 500ms

- [ ] **Recommendations < 1s (first load), < 100ms (cached)**
  - Load dashboard (first time)
  - Check `/api/v1/recommendations` timing
  - Refresh page (should hit cache)
  - Second load < 100ms

- [ ] **Progress tracking < 300ms**
  - Start a chapter
  - Check `/api/v1/progress/start` timing
  - Time < 300ms

- [ ] **RAG chatbot < 3s**
  - Ask a question
  - Measure time from send to response
  - Time < 3s

---

### 9. Error Handling

- [ ] **Unauthenticated access handled gracefully**
  - Sign out
  - Try to access `/api/v1/skill-level` directly
  - Redirected to login or shown friendly error

- [ ] **Invalid chapter ID handled**
  - Navigate to `/chapters/invalid-chapter-id`
  - 404 page shown or redirected gracefully

- [ ] **Network error handling**
  - Disconnect internet
  - Try to save profile
  - User-friendly error message shown
  - Reconnect internet
  - Functionality resumes

---

### 10. CORS Configuration

- [ ] **CORS headers present**
  - DevTools → Network tab
  - Check any API request
  - Response headers include:
    - `access-control-allow-origin: <frontend-url>`

- [ ] **Cross-origin requests work**
  - Frontend on staging-hackathon1.vercel.app
  - Backend on hackathon1-staging.up.railway.app
  - No CORS errors in browser console

---

## Test Results Summary

**Total Tests**: 50+
**Passed**: _____ / 50+
**Failed**: _____ / 50+

### Critical Issues Found
1. _____________________________________
2. _____________________________________
3. _____________________________________

### Notes
_____________________________________
_____________________________________
_____________________________________

### Recommendation
☐ **PASS** - Ready for UAT
☐ **CONDITIONAL PASS** - Minor issues, can proceed to UAT
☐ **FAIL** - Critical issues found, fix and re-test

---

**Tester Signature**: _____________________
**Date**: _____________________
**Time Spent**: _____ minutes
