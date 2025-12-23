---
id: 001
title: Phase 4B Personalization Engine Specification
stage: spec
date: 2025-12-22
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-personalization-engine
branch: 1-personalization-engine
user: Sajid Khan Afridi
command: /sp.specify
labels: ["personalization", "adaptive-learning", "recommendations", "skill-classification", "progress-tracking"]
links:
  spec: specs/1-personalization-engine/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-personalization-engine/spec.md
  - specs/1-personalization-engine/checklists/requirements.md
tests:
  - Specification quality checklist validation (PASSED)
---

## Prompt

"""Phase 4B: Personalization Engine

Build an adaptive learning system that personalizes the robotics textbook experience based on authenticated user profiles. This phase builds on Phase 4A's authentication and profile collection to deliver intelligent content recommendations and adaptive content presentation.

## Business Context

The Physical AI & Humanoid Robotics Textbook serves graduate-level students with varying backgrounds in programming, ROS, and hardware access. Phase 4A implemented authentication and collects 5 key profile attributes:
1. Programming experience level (beginner/intermediate/advanced)
2. ROS familiarity (none/basic/proficient)
3. Hardware access (simulation_only/jetson_kit/full_robot_lab)
4. Learning goal (career_transition/academic_research/hobby)
5. Preferred code language (python/cpp/both)

Phase 4B must use these profiles to personalize content delivery, recommend relevant chapters, and adapt content depth dynamically.

## Existing Infrastructure (DO NOT RECREATE)

The following components already exist and must be used as-is:
- `backend/app/services/profile_service.py` - ProfileService with get_profile, update_profile
- `backend/app/models/profile.py` - ProfileCreate, ProfileUpdate, ProfileResponse models
- `src/components/PersonalizedSection.tsx` - Conditional rendering based on user profile
- `src/context/UserContext.tsx` - User profile state management
- `src/components/Profile/ProfileWizard.tsx` - 5-question profile collection
- PersonalizedSection tags in all 10 MDX chapters (module-1, module-2, module-3)
- ADR-005: Section-level personalization via PersonalizedSection component

## Required Features

### 1. Skill Level Classification
Classify users into skill levels based on their profile answers to determine content adaptation depth.

### 2. Chapter Recommendations
Recommend the next best chapter to read based on:
- User's current skill level
- Learning goal
- Previously viewed chapters
- Content dependencies (prerequisites)

### 3. Learning Progress Tracking
Track which chapters users have started, completed, and bookmarked.

### 4. Adaptive Content Depth
The existing PersonalizedSection component shows/hides content based on profile. Enhance this to:
- Show appropriate code language examples (Python/C++/Both)
- Adjust exercise difficulty based on experience level
- Show hardware-specific content only for users with hardware access

### 5. Profile-Aware RAG Chatbot
When authenticated users ask questions, the chatbot should consider their profile:
- Adjust answer complexity to user's experience level
- Prioritize examples in user's preferred language
- Reference appropriate hardware context

## Integration Requirements

- Must integrate with existing Better Auth session management from Phase 4A
- Must read user profile from existing ProfileService
- Must NOT break existing PersonalizedSection functionality
- Must work within free tier limits (Neon 0.5GB, OpenAI rate limits)
- Frontend must remain statically deployable to GitHub Pages

## Quality Gates (from constitution.md)

- Personalization response time < 2s (measured end-to-end)
- Recommendation relevance > 0.75 (validated via user feedback or A/B test)
- Skill level classification accuracy > 80%

## Out of Scope

- Email notifications about new chapters
- Social features (sharing progress)
- Gamification (badges, leaderboards)
- Offline mode / PWA features
- Admin dashboard for content personalization rules

## Constraints

- Use existing Neon Postgres for all new data storage
- No new external services (stay within free tier)
- Backend changes must maintain Railway deployment compatibility
- Frontend changes must work with static GitHub Pages deployment
"""

## Response snapshot

Created comprehensive specification for Phase 4B Personalization Engine with:

**User Stories (Prioritized):**
1. **P1 - Skill Level Classification**: Foundation for all personalization features
2. **P2 - Chapter Progress Tracking**: Enables recommendation relevance
3. **P3 - Smart Chapter Recommendations**: Depends on P1 and P2
4. **P2 - Adaptive Content Depth**: Enhances existing PersonalizedSection component
5. **P3 - Profile-Aware RAG Chatbot**: Enhances existing chatbot with profile awareness

**Functional Requirements:**
- 30 testable functional requirements (FR-001 to FR-030) covering:
  - Skill level classification algorithm (FR-001 to FR-004)
  - Chapter progress tracking (FR-005 to FR-011)
  - Chapter recommendations with prerequisites (FR-012 to FR-017)
  - Adaptive content filtering (FR-018 to FR-021)
  - Profile-aware chatbot responses (FR-022 to FR-026)
  - Integration with Phase 4A infrastructure (FR-027 to FR-030)

**Success Criteria:**
- 8 measurable, technology-agnostic outcomes (SC-001 to SC-008)
- Key metrics: <2s response time, >0.75 relevance score, >80% classification accuracy, >85% chatbot satisfaction

**Key Entities:**
- SkillLevelClassification (user skill tier)
- ChapterProgress (started/completed/bookmarked state)
- ChapterRecommendation (personalized suggestions with relevance scores)
- ChapterMetadata (difficulty, prerequisites, hardware requirements)

**Edge Cases Addressed:**
- Profile updates triggering re-classification
- Partial profile completion with fallback defaults
- Circular dependency detection in prerequisites
- Tie-breaking in recommendations

**Files Created:**
- `specs/1-personalization-engine/spec.md` (complete specification)
- `specs/1-personalization-engine/checklists/requirements.md` (validation checklist - all items PASSED)

## Outcome

- ✅ Impact: Comprehensive specification for adaptive learning personalization system with clear prioritization, testable requirements, and measurable success criteria
- 🧪 Tests: Specification quality checklist validated all items (no [NEEDS CLARIFICATION] markers, all requirements testable, success criteria measurable)
- 📁 Files: Created spec.md (5 user stories, 30 functional requirements, 8 success criteria, 6 edge cases) and requirements.md checklist
- 🔁 Next prompts: `/sp.plan` to design implementation architecture; `/sp.clarify` optional but spec is already very complete
- 🧠 Reflection: Strong specification with clear dependencies on Phase 4A infrastructure, realistic constraints for free tier deployment, and independent testability for each user story

## Evaluation notes (flywheel)

- Failure modes observed: None - specification validation passed all checks
- Graders run and results (PASS/FAIL): Specification Quality Checklist - PASSED (all 13 validation items met)
- Prompt variant (if applicable): Standard /sp.specify workflow with phase-based feature numbering
- Next experiment (smallest change to try): Proceed directly to `/sp.plan` given zero [NEEDS CLARIFICATION] markers and complete validation
