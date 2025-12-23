# Phase 4B User Acceptance Testing (UAT) Plan

**Feature**: Personalization Engine
**Environment**: Staging
**Testing Period**: 2-3 days after staging deployment
**Estimated Testing Time**: 3-4 hours per tester

---

## UAT Objectives

Validate that Phase 4B Personalization Engine:
1. ✅ Correctly classifies users into skill levels
2. ✅ Provides relevant chapter recommendations
3. ✅ Tracks chapter progress accurately
4. ✅ Delivers personalized content based on user profile
5. ✅ Integrates seamlessly with RAG chatbot
6. ✅ Meets performance requirements (< 2s response time)
7. ✅ Is WCAG 2.1 AA accessible

---

## Test Environment

- **Frontend**: https://staging-hackathon1.vercel.app
- **Backend**: https://hackathon1-staging.up.railway.app
- **Test Accounts**: Use personal Google/GitHub accounts
- **Test Duration**: 3-4 hours
- **Browsers**: Chrome, Firefox, Safari, Edge

---

## Test Scenarios

### Scenario 1: New User Onboarding (Beginner Profile)

**Objective**: Verify beginner user gets appropriate personalized experience

**Steps**:
1. Navigate to staging frontend
2. Click "Sign In" → "Continue with Google"
3. Sign in with Google account (first time user)
4. Complete profile settings:
   - Programming Experience: **Beginner**
   - ROS Familiarity: **None**
   - Preferred Language: **Python**
   - Hardware Access: **Simulation Only**
   - Learning Goal: **Academic Research**
5. Click "Save Profile"

**Expected Results**:
- [ ] Profile saved successfully
- [ ] Redirected to dashboard/home page
- [ ] Skill level badge shows **"Beginner"**
- [ ] Recommended chapters displayed (should be Module 1 beginner chapters)
- [ ] Content shows Python code examples only
- [ ] No hardware-specific instructions shown
- [ ] Chatbot responses are beginner-friendly

**Performance**:
- [ ] Profile save completes in < 500ms
- [ ] Skill classification completes in < 500ms
- [ ] Recommendations load in < 1s

---

### Scenario 2: Advanced User Profile

**Objective**: Verify advanced user gets appropriate personalized experience

**Steps**:
1. Sign out (if signed in)
2. Sign in with different Google account
3. Complete profile settings:
   - Programming Experience: **Advanced**
   - ROS Familiarity: **Proficient**
   - Preferred Language: **C++**
   - Hardware Access: **Physical Robot**
   - Learning Goal: **Research**
4. Click "Save Profile"

**Expected Results**:
- [ ] Skill level badge shows **"Advanced"**
- [ ] Recommended chapters include Module 3 advanced topics
- [ ] Content shows C++ code examples only
- [ ] Hardware setup instructions visible
- [ ] Chatbot responses assume advanced knowledge

**Test Cases**:
- [ ] Navigate to "Advanced Robot Control" chapter → Should be visible
- [ ] Navigate to "ROS Intro" chapter → Should still be accessible
- [ ] Check code examples → Should show C++ by default

---

### Scenario 3: Chapter Progress Tracking

**Objective**: Verify chapter progress tracking works correctly

**Steps**:
1. Sign in as beginner user
2. Navigate to "Introduction to ROS 2" chapter
3. Wait 10 seconds (auto-start threshold)
4. Check progress indicator
5. Scroll to bottom of chapter
6. Check completion status
7. Click bookmark icon
8. Navigate to profile/dashboard
9. Check progress summary

**Expected Results**:
- [ ] Chapter marked as "Started" after 10 seconds
- [ ] Progress indicator shows "In Progress"
- [ ] Chapter marked as "Completed" when scrolled to bottom
- [ ] Completion checkmark visible
- [ ] Bookmark icon toggles on/off
- [ ] Dashboard shows:
  - [ ] 1 chapter started
  - [ ] 1 chapter completed
  - [ ] 1 bookmarked chapter

**API Validation**:
```bash
# Verify progress in API
curl -X GET "https://hackathon1-staging.up.railway.app/api/v1/progress" \
  -H "Authorization: Bearer YOUR_TOKEN"

# Expected response shows chapter_id with status "completed"
```

---

### Scenario 4: Smart Chapter Recommendations

**Objective**: Verify recommendation algorithm provides relevant suggestions

**Steps**:
1. Sign in as intermediate user:
   - Programming Experience: **Intermediate**
   - ROS Familiarity: **Basic**
   - Learning Goal: **Career Transition**
2. Navigate to dashboard/home page
3. Check "Recommended for You" section

**Expected Results**:
- [ ] 3 chapter recommendations displayed
- [ ] Each recommendation has:
  - [ ] Chapter title
  - [ ] Relevance score (displayed or indicated)
  - [ ] Reason for recommendation
  - [ ] "Start Learning" button
- [ ] Recommendations match intermediate skill level
- [ ] No beginner-only or advanced-only chapters recommended
- [ ] Recommendations consider learning goal (practical/career-focused)

**Test Variations**:
- [ ] Complete a recommended chapter → Recommendations should update
- [ ] Change profile settings → Recommendations should refresh
- [ ] Wait 1 hour → Recommendations should come from cache (faster load)

---

### Scenario 5: Profile-Aware RAG Chatbot

**Objective**: Verify chatbot provides personalized, contextual answers

**Test Case 5A: Beginner User Query**

**Steps**:
1. Sign in as beginner user
2. Open chatbot
3. Ask: "How do I create a ROS publisher?"

**Expected Results**:
- [ ] Response includes:
  - [ ] Python code example (not C++)
  - [ ] Beginner-friendly explanation
  - [ ] Simulation-based instructions
  - [ ] No assumptions about advanced ROS knowledge
- [ ] Response time < 3s

**Test Case 5B: Advanced User Query**

**Steps**:
1. Sign in as advanced user
2. Open chatbot
3. Ask: "How do I create a ROS publisher?"

**Expected Results**:
- [ ] Response includes:
  - [ ] C++ code example (not Python)
  - [ ] Advanced details (QoS policies, lifecycle nodes, etc.)
  - [ ] Hardware-specific considerations
  - [ ] Assumes knowledge of ROS fundamentals

**Test Case 5C: Context Preservation**

**Steps**:
1. Ask: "What is inverse kinematics?"
2. Follow up: "Can you show me a code example?"

**Expected Results**:
- [ ] Second response understands context (inverse kinematics)
- [ ] Code example matches user's preferred language
- [ ] Conversation history maintained

---

### Scenario 6: Adaptive Content Filtering

**Objective**: Verify content adapts to user profile

**Steps**:
1. Sign in as Python beginner user
2. Navigate to "Creating ROS 2 Publishers" chapter
3. Check code examples
4. Sign out
5. Sign in as C++ advanced user
6. Navigate to same chapter
7. Compare content

**Expected Results - Beginner User**:
- [ ] Python code examples shown
- [ ] C++ code examples hidden or collapsed
- [ ] Beginner explanations visible
- [ ] Advanced sections hidden

**Expected Results - Advanced User**:
- [ ] C++ code examples shown
- [ ] Python code examples hidden or collapsed
- [ ] Advanced sections visible
- [ ] Beginner explanations collapsed

**Edge Cases**:
- [ ] User with "Both" language preference → Both code examples shown
- [ ] Unauthenticated user → Default intermediate Python shown

---

### Scenario 7: Performance Testing

**Objective**: Verify all endpoints meet performance requirements

**Test Cases**:
- [ ] **Skill Classification**: < 500ms
  - Navigate to profile → Check skill level badge load time

- [ ] **Recommendations**: < 1s (cached), < 2s (uncached)
  - Refresh dashboard → Measure recommendations load time

- [ ] **Progress Tracking**: < 300ms
  - Start a chapter → Measure time to "Started" status

- [ ] **RAG Chatbot**: < 3s
  - Ask a question → Measure response time

**Validation**:
```bash
# Use browser DevTools → Network tab
# Check timing for API calls:
# - /api/v1/skill-level
# - /api/v1/recommendations
# - /api/v1/progress/start
# - /api/v1/chat/query
```

---

### Scenario 8: Accessibility Testing

**Objective**: Verify WCAG 2.1 AA compliance

**Tools**:
- Lighthouse (Chrome DevTools)
- axe DevTools browser extension
- Keyboard navigation

**Test Cases**:
- [ ] **Keyboard Navigation**:
  - [ ] Tab through profile settings form
  - [ ] Navigate to chapters using keyboard only
  - [ ] Activate chatbot using keyboard

- [ ] **Screen Reader** (NVDA/JAWS/VoiceOver):
  - [ ] Skill level badge announced correctly
  - [ ] Recommendations have descriptive labels
  - [ ] Progress indicators announced

- [ ] **Color Contrast**:
  - [ ] Skill level badges meet contrast requirements
  - [ ] Recommendation cards readable
  - [ ] Progress indicators distinguishable

**Lighthouse Checks**:
```bash
# Run Lighthouse audit
npm run lighthouse

# Target scores:
# - Accessibility: 100
# - Performance: 90+
# - Best Practices: 100
# - SEO: 90+
```

---

### Scenario 9: Edge Cases and Error Handling

**Test Cases**:

**TC9.1: Incomplete Profile**
- [ ] Skip profile completion → Should use default intermediate profile
- [ ] Chatbot still works with default profile

**TC9.2: Network Errors**
- [ ] Disconnect internet → Error message shown gracefully
- [ ] Reconnect → Resume functionality

**TC9.3: Invalid Chapter ID**
- [ ] Navigate to non-existent chapter → 404 page shown
- [ ] Try to track progress on invalid chapter → Error handled

**TC9.4: Session Expiration**
- [ ] Wait for JWT expiration (24 hours) → Should redirect to login
- [ ] Sign in again → Resume where left off

**TC9.5: Concurrent Progress Updates**
- [ ] Open same chapter in 2 tabs
- [ ] Complete in one tab → Other tab should update

---

## Bug Reporting Template

If you encounter issues, please report using this template:

```markdown
### Bug Report: [Brief Description]

**Severity**: Critical / High / Medium / Low

**Test Scenario**: [Scenario number]

**Steps to Reproduce**:
1.
2.
3.

**Expected Result**:
[What should happen]

**Actual Result**:
[What actually happened]

**Screenshots/Videos**:
[Attach if applicable]

**Browser/Device**:
- Browser: Chrome 120.0
- OS: Windows 11
- Screen size: 1920x1080

**Console Errors**:
```
[Paste console errors here]
```

**Network Logs**:
[Attach HAR file or screenshots of Network tab]

**Reproducibility**: Always / Sometimes / Once
```

---

## UAT Sign-Off Criteria

Phase 4B is **approved for production** if:

- [ ] **Functional Testing**: All 9 scenarios pass with no critical bugs
- [ ] **Performance**: All endpoints meet performance requirements
- [ ] **Accessibility**: Lighthouse accessibility score ≥ 100
- [ ] **Browser Compatibility**: Works on Chrome, Firefox, Safari, Edge
- [ ] **Mobile Responsive**: Works on mobile devices (bonus)
- [ ] **No Critical Bugs**: 0 severity-critical or severity-high bugs
- [ ] **Stakeholder Approval**: Product owner signs off

---

## UAT Completion Checklist

After testing, complete this checklist:

- [ ] All 9 test scenarios executed
- [ ] Bug reports filed for any issues found
- [ ] Performance benchmarks documented
- [ ] Accessibility audit completed
- [ ] Browser compatibility verified
- [ ] Test evidence collected (screenshots/videos)
- [ ] UAT report created
- [ ] Stakeholder notified of results

---

## UAT Report Template

```markdown
# Phase 4B UAT Report

**Tester**: [Your Name]
**Date**: [Date]
**Duration**: [X hours]
**Environment**: Staging

## Summary
[Overall impression of the feature]

## Test Results
- Scenarios Passed: X/9
- Scenarios Failed: X/9
- Critical Bugs: X
- High Priority Bugs: X
- Medium/Low Bugs: X

## Performance Results
- Skill Classification: Xms (Target: <500ms)
- Recommendations: Xms (Target: <1000ms)
- Progress Tracking: Xms (Target: <300ms)
- RAG Chatbot: Xms (Target: <3000ms)

## Accessibility Results
- Lighthouse Accessibility Score: X/100
- Keyboard Navigation: Pass/Fail
- Screen Reader: Pass/Fail

## Critical Issues
1. [Issue description with severity]
2. [Issue description with severity]

## Recommendation
☐ **Approve for Production**
☐ **Fix Issues and Retest**
☐ **Reject - Major Issues Found**

## Notes
[Additional observations]
```

---

**Document Version**: 1.0
**Last Updated**: 2025-12-23
**Next Review**: After UAT completion
