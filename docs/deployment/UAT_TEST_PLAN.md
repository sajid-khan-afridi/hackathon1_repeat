# User Acceptance Testing (UAT) Plan - Phase 4B Personalization Engine

**Feature**: Phase 4B Personalization Engine
**Version**: 1.0
**Environment**: Staging
**UAT Duration**: 2-3 days
**Target Audience**: Product Owner, QA Team, Test Users
**Date**: 2025-12-23

---

## Executive Summary

This UAT plan validates the Phase 4B Personalization Engine features work correctly from an end-user perspective in the staging environment before production deployment.

**Phase 4B Features Under Test**:
1. ✅ Rule-based skill level classification
2. ✅ Personalized chapter recommendations
3. ✅ Progress tracking (start, complete, bookmark)
4. ✅ RAG chatbot personalization based on user profile and progress
5. ✅ User profile management with Phase 4B fields

---

## UAT Objectives

### Primary Objectives
- Validate all Phase 4B features meet functional requirements
- Ensure personalization improves user learning experience
- Verify data accuracy and persistence
- Confirm performance meets expectations
- Identify any usability issues before production

### Success Criteria
- ✅ All critical user scenarios pass
- ✅ No blocker or critical bugs found
- ✅ Performance acceptable (API < 1s, page load < 3s)
- ✅ User experience deemed intuitive
- ✅ Product Owner sign-off obtained

---

## Test Environment

### Access Information

**Frontend Staging**: `https://hackathon1-staging-abc123.vercel.app`
**Backend API**: `https://hackathon1-staging.up.railway.app`
**API Docs**: `https://hackathon1-staging.up.railway.app/api/docs`

### Test Accounts

Create 3 test accounts representing different user personas:

#### Test Account 1: Beginner User
- **Email**: test.beginner@staging.test
- **Profile**:
  - ROS Experience: None
  - Python Proficiency: Beginner
  - Hardware Access: No
  - Learning Goals: Theoretical, Practical
- **Expected Skill Level**: Beginner
- **Expected Recommendations**: Module 1 chapters

#### Test Account 2: Intermediate User
- **Email**: test.intermediate@staging.test
- **Profile**:
  - ROS Experience: Beginner
  - Python Proficiency: Intermediate
  - Hardware Access: No
  - Learning Goals: Practical, Research
- **Expected Skill Level**: Intermediate
- **Expected Recommendations**: Module 2 chapters

#### Test Account 3: Advanced User
- **Email**: test.advanced@staging.test
- **Profile**:
  - ROS Experience: Intermediate
  - Python Proficiency: Advanced
  - Hardware Access: Yes
  - Learning Goals: Research, Hardware
- **Expected Skill Level**: Advanced
- **Expected Recommendations**: Module 3 chapters

---

## UAT Test Cases

### Feature 1: User Profile Management

#### TC1.1: Create User Profile (First-Time User)

**Persona**: Beginner User

**Preconditions**:
- User not logged in
- No existing profile

**Steps**:
1. Navigate to staging frontend homepage
2. Click "Login with Google" or "Login with GitHub"
3. Complete OAuth flow
4. Redirected to profile setup page
5. Fill out profile form:
   - Native Language: English
   - ROS Experience: None
   - Python Proficiency: Beginner
   - Hardware Access: No
   - Learning Goals: Select "Theoretical" and "Practical"
6. Click "Save Profile"

**Expected Results**:
- ✅ OAuth login successful
- ✅ Profile form displays with all Phase 4B fields
- ✅ Profile saves successfully
- ✅ Success message displayed
- ✅ User redirected to personalized homepage

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

**Notes**: ______________________

---

#### TC1.2: Update User Profile

**Persona**: Intermediate User

**Preconditions**:
- User logged in
- Profile already exists

**Steps**:
1. Navigate to `/profile` page
2. Update Python Proficiency: Intermediate → Advanced
3. Update Hardware Access: No → Yes
4. Add new learning goal: "Hardware"
5. Click "Save Changes"
6. Refresh page

**Expected Results**:
- ✅ Profile form pre-filled with existing data
- ✅ Changes save successfully
- ✅ Updated values persist after page refresh
- ✅ Skill level recalculates (may change to Advanced)

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

### Feature 2: Skill Level Classification

#### TC2.1: Skill Level Calculation (Beginner)

**Persona**: Beginner User

**Preconditions**:
- Profile created with beginner settings (TC1.1)

**Steps**:
1. Navigate to `/profile` page
2. Observe skill level badge/indicator

**Expected Results**:
- ✅ Skill level badge displays "Beginner"
- ✅ Badge styled appropriately (color, icon)
- ✅ Tooltip or description explains skill level

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

#### TC2.2: Skill Level Calculation (Intermediate)

**Persona**: Intermediate User

**Preconditions**:
- Profile created with intermediate settings

**Steps**:
1. Navigate to `/profile` page
2. Observe skill level badge

**Expected Results**:
- ✅ Skill level badge displays "Intermediate"
- ✅ Badge reflects intermediate status visually

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

#### TC2.3: Skill Level Calculation (Advanced)

**Persona**: Advanced User

**Preconditions**:
- Profile created with advanced settings

**Steps**:
1. Navigate to `/profile` page
2. Observe skill level badge

**Expected Results**:
- ✅ Skill level badge displays "Advanced"
- ✅ Badge reflects advanced status visually

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

#### TC2.4: Skill Level Recalculation on Profile Update

**Persona**: Beginner User

**Preconditions**:
- Current skill level: Beginner

**Steps**:
1. Navigate to `/profile`
2. Current skill level: Beginner (displayed)
3. Update profile:
   - Change Python Proficiency: Beginner → Advanced
   - Change ROS Experience: None → Beginner
4. Save changes
5. Observe skill level badge

**Expected Results**:
- ✅ Skill level recalculates automatically
- ✅ Badge updates to "Intermediate" or "Advanced"
- ✅ No page refresh required (updates immediately)

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

### Feature 3: Personalized Chapter Recommendations

#### TC3.1: Beginner Recommendations

**Persona**: Beginner User

**Preconditions**:
- Skill level: Beginner
- No chapters started

**Steps**:
1. Navigate to homepage
2. Locate "Recommended for You" section
3. Review recommended chapters

**Expected Results**:
- ✅ "Recommended for You" section visible
- ✅ Recommendations include Module 1 (beginner) chapters:
  - module-1/ros-intro
  - module-1/linux-basics
  - module-1/python-basics
  - module-1/gazebo-simulation
- ✅ No Module 3 (advanced) chapters recommended
- ✅ No hardware-required chapters (user has no hardware)
- ✅ Chapters sorted by relevance or module order

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

#### TC3.2: Intermediate Recommendations

**Persona**: Intermediate User

**Preconditions**:
- Skill level: Intermediate
- Module 1 chapters completed

**Steps**:
1. Navigate to homepage
2. Review recommended chapters

**Expected Results**:
- ✅ Recommendations include Module 2 (intermediate) chapters:
  - module-2/ros-publishers
  - module-2/ros-subscribers
  - module-2/ros-services
- ✅ No beginner-only chapters recommended
- ✅ Recommendations respect prerequisites (show only unlocked chapters)

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

#### TC3.3: Advanced Recommendations with Hardware

**Persona**: Advanced User

**Preconditions**:
- Skill level: Advanced
- Hardware Access: Yes

**Steps**:
1. Navigate to homepage
2. Review recommended chapters

**Expected Results**:
- ✅ Recommendations include Module 3 (advanced) chapters:
  - module-3/advanced-control
  - module-3/inverse-kinematics
  - module-3/isaac-integration
- ✅ Hardware-required chapters included (user has hardware)
- ✅ No beginner chapters recommended

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

#### TC3.4: Recommendations Update After Progress

**Persona**: Beginner User

**Preconditions**:
- Initial recommendations: Module 1 chapters

**Steps**:
1. Note current recommendations
2. Complete a recommended chapter (e.g., module-1/ros-intro)
3. Return to homepage
4. Check recommendations

**Expected Results**:
- ✅ Completed chapter no longer in recommendations
- ✅ Next logical chapter recommended (e.g., module-1/linux-basics)
- ✅ Recommendations reflect updated progress

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

### Feature 4: Progress Tracking

#### TC4.1: Start Chapter

**Persona**: Beginner User

**Preconditions**:
- No progress on any chapters

**Steps**:
1. Navigate to homepage
2. Click "Start Chapter" on recommended chapter (module-1/ros-intro)
3. Chapter page loads
4. Navigate to `/profile` → "Your Progress" section

**Expected Results**:
- ✅ Chapter opens successfully
- ✅ "Start Chapter" button changes to "Continue" or "Resume"
- ✅ Progress saved to database (status: started)
- ✅ Progress displays in profile:
  - Chapter title listed under "In Progress"
  - Started timestamp visible
- ✅ Progress indicator shows 0% or "Started"

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

#### TC4.2: Complete Chapter

**Persona**: Beginner User

**Preconditions**:
- Chapter module-1/ros-intro started (TC4.1)

**Steps**:
1. Navigate to chapter page (module-1/ros-intro)
2. Read through chapter content
3. Click "Mark as Complete" button
4. Navigate to `/profile`

**Expected Results**:
- ✅ Completion confirmation message displayed
- ✅ Button changes to "Completed" (disabled or different style)
- ✅ Database updated: status='completed', completed_at set
- ✅ Profile shows chapter under "Completed" section
- ✅ Completion timestamp visible
- ✅ Progress indicator shows 100% or "Completed"

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

#### TC4.3: Bookmark Chapter

**Persona**: Intermediate User

**Preconditions**:
- Viewing chapter module-2/ros-publishers

**Steps**:
1. Navigate to chapter page
2. Click "Bookmark" icon/button
3. Navigate to `/profile` → "Bookmarks" section

**Expected Results**:
- ✅ Bookmark icon changes state (filled/highlighted)
- ✅ Confirmation message or visual feedback
- ✅ Database updated: is_bookmarked=true
- ✅ Chapter appears in "Bookmarks" section on profile
- ✅ Clicking bookmark again unbookmarks (toggle behavior)

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

#### TC4.4: Progress Persistence

**Persona**: Any User

**Preconditions**:
- User has started/completed multiple chapters

**Steps**:
1. Start chapter A
2. Complete chapter B
3. Bookmark chapter C
4. Logout
5. Login again
6. Navigate to `/profile`

**Expected Results**:
- ✅ All progress persists after logout/login
- ✅ Started chapters still show "In Progress"
- ✅ Completed chapters still show "Completed"
- ✅ Bookmarks still appear in bookmarks section
- ✅ Timestamps accurate

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

#### TC4.5: Multiple Progress Records

**Persona**: Intermediate User

**Preconditions**:
- No progress on any chapters

**Steps**:
1. Start chapter module-2/ros-publishers
2. Start chapter module-2/ros-subscribers
3. Complete chapter module-2/ros-publishers
4. View profile progress section

**Expected Results**:
- ✅ Both chapters appear in progress list
- ✅ Completed chapter shows completed status
- ✅ Started chapter shows started status
- ✅ Each has correct timestamp
- ✅ Progress displayed in chronological or logical order

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

### Feature 5: RAG Chatbot Personalization

#### TC5.1: Chatbot Responds to Beginner User

**Persona**: Beginner User

**Preconditions**:
- User logged in
- Skill level: Beginner
- No chapters started

**Steps**:
1. Navigate to `/chatbot`
2. Ask: "What is ROS 2?"
3. Review response

**Expected Results**:
- ✅ Chatbot responds with beginner-friendly explanation
- ✅ Response uses simple language, minimal jargon
- ✅ Response may mention "new to ROS" or "beginner level"
- ✅ Response includes basic concepts
- ✅ Response does not assume prior knowledge

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

#### TC5.2: Chatbot Responds to Advanced User

**Persona**: Advanced User

**Preconditions**:
- User logged in
- Skill level: Advanced

**Steps**:
1. Navigate to `/chatbot`
2. Ask: "Explain the DDS middleware architecture in ROS 2"
3. Review response

**Expected Results**:
- ✅ Chatbot responds with technical depth
- ✅ Response includes advanced concepts (QoS, discovery, etc.)
- ✅ Response assumes prior knowledge
- ✅ Response may reference user's advanced skill level

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

#### TC5.3: Chatbot Context Includes User Progress

**Persona**: Beginner User

**Preconditions**:
- User started chapter module-1/ros-intro
- User completed chapter module-1/python-basics

**Steps**:
1. Navigate to `/chatbot`
2. Ask: "What should I learn next?"
3. Review response

**Expected Results**:
- ✅ Response acknowledges started/completed chapters
- ✅ Response suggests next logical chapter in learning path
- ✅ Response considers prerequisites met
- ✅ Response may mention "Since you've completed X, try Y"

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

#### TC5.4: Chatbot Respects Hardware Limitations

**Persona**: Intermediate User (No Hardware)

**Preconditions**:
- Hardware Access: No

**Steps**:
1. Navigate to `/chatbot`
2. Ask: "How do I control a physical robot arm?"
3. Review response

**Expected Results**:
- ✅ Response suggests simulation alternatives (Gazebo, Isaac Sim)
- ✅ Response may mention "Since you don't have hardware access..."
- ✅ Response provides workarounds or virtual options

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

#### TC5.5: Chatbot Chat History Persistence

**Persona**: Any User

**Preconditions**:
- User logged in

**Steps**:
1. Navigate to `/chatbot`
2. Ask question: "What is a ROS node?"
3. Receive response
4. Ask follow-up: "Can you give an example?"
5. Refresh page
6. Check chat history

**Expected Results**:
- ✅ Follow-up question uses context from previous question
- ✅ Chat history persists after page refresh
- ✅ Previous conversation visible in UI
- ✅ Session ID maintained

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

### Feature 6: User Experience & Usability

#### TC6.1: Responsive Design

**Persona**: Any User

**Steps**:
1. Open staging frontend on desktop (1920x1080)
2. Open staging frontend on tablet (768x1024)
3. Open staging frontend on mobile (375x667)
4. Test key pages: homepage, profile, chatbot, chapter

**Expected Results**:
- ✅ All pages responsive on all screen sizes
- ✅ No horizontal scrolling
- ✅ Content readable and accessible
- ✅ Buttons/links large enough to tap on mobile
- ✅ Navigation menu collapses appropriately

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

#### TC6.2: Dark Mode Toggle

**Persona**: Any User

**Steps**:
1. Open staging frontend
2. Toggle dark mode switch (if available)
3. Navigate between pages

**Expected Results**:
- ✅ Dark mode applies consistently across all pages
- ✅ Preference persists after page refresh
- ✅ All text remains readable in dark mode
- ✅ No contrast issues

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

#### TC6.3: Error Handling & User Feedback

**Persona**: Any User

**Steps**:
1. Try starting a chapter without logging in
2. Try updating profile with invalid data
3. Try bookmarking a non-existent chapter
4. Disconnect internet, try chatbot query

**Expected Results**:
- ✅ Clear error messages displayed
- ✅ User redirected to login when required
- ✅ Form validation errors shown
- ✅ Network errors handled gracefully
- ✅ No application crashes

**Actual Results**: ______________________

**Status**: ☐ Pass ☐ Fail

---

### Feature 7: Performance

#### TC7.1: Homepage Load Time

**Persona**: Any User

**Steps**:
1. Clear browser cache
2. Navigate to homepage
3. Measure load time (Network tab → DOMContentLoaded)

**Expected Results**:
- ✅ DOMContentLoaded < 2 seconds
- ✅ Fully loaded < 3 seconds
- ✅ No blocking resources

**Actual Results**: ______________________

**Load Time**: ________ ms

**Status**: ☐ Pass ☐ Fail

---

#### TC7.2: Recommendations API Response Time

**Persona**: Any User (logged in)

**Steps**:
1. Open browser DevTools → Network tab
2. Navigate to homepage
3. Find API call to `/api/v1/recommendations`
4. Check response time

**Expected Results**:
- ✅ Response time < 1000ms
- ✅ No timeouts

**Actual Results**: ______________________

**Response Time**: ________ ms

**Status**: ☐ Pass ☐ Fail

---

#### TC7.3: Chatbot Response Time

**Persona**: Any User

**Steps**:
1. Navigate to `/chatbot`
2. Ask simple question: "What is ROS?"
3. Measure time from send to response display

**Expected Results**:
- ✅ Response displayed < 3 seconds
- ✅ Loading indicator shows during processing

**Actual Results**: ______________________

**Response Time**: ________ seconds

**Status**: ☐ Pass ☐ Fail

---

## UAT Execution Schedule

### Day 1: Core Functionality

**Morning (2-3 hours)**:
- Set up test accounts (3 personas)
- Execute Feature 1: User Profile Management (TC1.1-1.2)
- Execute Feature 2: Skill Level Classification (TC2.1-2.4)

**Afternoon (2-3 hours)**:
- Execute Feature 3: Personalized Recommendations (TC3.1-3.4)
- Execute Feature 4: Progress Tracking (TC4.1-4.5)

**End of Day**:
- Document any issues found
- Create bug tickets for failures

---

### Day 2: Personalization & UX

**Morning (2-3 hours)**:
- Execute Feature 5: RAG Chatbot Personalization (TC5.1-5.5)
- Execute Feature 6: User Experience & Usability (TC6.1-6.3)

**Afternoon (2 hours)**:
- Execute Feature 7: Performance (TC7.1-7.3)
- Regression test any bug fixes from Day 1

**End of Day**:
- Compile test results
- Prepare UAT report

---

### Day 3: Final Validation & Sign-off

**Morning (1-2 hours)**:
- Retest any failed cases
- Exploratory testing (edge cases)
- Final verification of bug fixes

**Afternoon (1 hour)**:
- UAT sign-off meeting with Product Owner
- Document production deployment readiness

---

## Bug Reporting Template

### Bug Report Format

```
Bug ID: UAT-4B-XXX
Severity: ☐ Blocker ☐ Critical ☐ Major ☐ Minor
Status: ☐ Open ☐ In Progress ☐ Fixed ☐ Verified

Title: [Brief description]

Test Case: TCXX.X - [Test case name]

Steps to Reproduce:
1. [Step 1]
2. [Step 2]
3. [Step 3]

Expected Result:
[What should happen]

Actual Result:
[What actually happened]

Environment:
- Frontend: https://hackathon1-staging-abc123.vercel.app
- Backend: https://hackathon1-staging.up.railway.app
- Browser: [Chrome/Firefox/Safari]
- User Persona: [Beginner/Intermediate/Advanced]

Screenshots/Logs:
[Attach if applicable]

Reported By: [Name]
Reported On: [Date]
```

---

## UAT Results Summary

### Test Execution Summary

| Feature | Total TCs | Passed | Failed | Pass Rate |
|---------|-----------|--------|--------|-----------|
| 1. User Profile Management | 2 | ___ | ___ | ___% |
| 2. Skill Level Classification | 4 | ___ | ___ | ___% |
| 3. Personalized Recommendations | 4 | ___ | ___ | ___% |
| 4. Progress Tracking | 5 | ___ | ___ | ___% |
| 5. RAG Chatbot Personalization | 5 | ___ | ___ | ___% |
| 6. User Experience & Usability | 3 | ___ | ___ | ___% |
| 7. Performance | 3 | ___ | ___ | ___% |
| **TOTAL** | **26** | ___ | ___ | ___% |

### Defect Summary

| Severity | Count | Examples |
|----------|-------|----------|
| Blocker | ___ | [List blocker bugs] |
| Critical | ___ | [List critical bugs] |
| Major | ___ | [List major bugs] |
| Minor | ___ | [List minor bugs] |

### Overall Assessment

- [ ] **APPROVED**: Ready for production deployment
- [ ] **APPROVED WITH CONDITIONS**: Deploy with documented workarounds
- [ ] **REJECTED**: Critical issues must be fixed before production

### Conditions for Approval (if applicable)

1. ______________________
2. ______________________

---

## UAT Sign-off

### Test Team Sign-off

**QA Lead**: ______________________
**Date**: ______________________
**Signature**: ______________________

**Test User 1 (Beginner Persona)**: ______________________
**Date**: ______________________

**Test User 2 (Intermediate Persona)**: ______________________
**Date**: ______________________

**Test User 3 (Advanced Persona)**: ______________________
**Date**: ______________________

---

### Product Owner Sign-off

**Name**: ______________________
**Date**: ______________________
**Signature**: ______________________

**Decision**: ☐ APPROVED FOR PRODUCTION  ☐ REQUIRES FIXES

**Comments**: ______________________

---

## Post-UAT Actions

### If Approved for Production

1. [ ] Schedule production deployment window
2. [ ] Prepare production environment variables
3. [ ] Create production database backup
4. [ ] Update production OAuth apps
5. [ ] Execute production deployment (see `STAGING_DEPLOYMENT_RUNBOOK.md`)
6. [ ] Run smoke tests on production
7. [ ] Monitor production for 24 hours post-deployment

### If Rejected

1. [ ] Fix all blocker and critical bugs
2. [ ] Redeploy to staging
3. [ ] Execute focused regression UAT (retest failed cases)
4. [ ] Obtain sign-off before production deployment

---

**Document Version**: 1.0
**Last Updated**: 2025-12-23
**Maintained By**: Product Owner & QA Team
