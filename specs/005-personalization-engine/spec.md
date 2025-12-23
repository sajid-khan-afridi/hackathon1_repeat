# Feature Specification: Phase 4B Personalization Engine

**Feature Branch**: `1-personalization-engine`
**Created**: 2025-12-22
**Status**: Draft
**Input**: User description: "Phase 4B: Personalization Engine - Build an adaptive learning system that personalizes the robotics textbook experience based on authenticated user profiles. This phase builds on Phase 4A's authentication and profile collection to deliver intelligent content recommendations and adaptive content presentation."

<!--
IMPORTANT: Branch naming follows phase-based numbering from the constitution:
- Phase 1 (Book Infrastructure): 001-*
- Phase 2 (Content Creation): 002-*
- Phase 3 (RAG Chatbot Core): 003-*
- Phase 4A (Authentication): 004-*
- Phase 4B (Personalization): 005-*
- Phase 5 (Translation): 006-*
- Phase 6 (Integration & Deployment): 007-*

Examples: 001-docusaurus-book-infra, 002-mdx-textbook-chapters, 003-rag-chatbot-core
-->

## Clarifications

### Session 2025-12-22

- Q: Should the database enforce uniqueness on (user_id, chapter_id) for ChapterProgress, or allow multiple progress records per chapter? → A: Enforce unique constraint on (user_id, chapter_id) - only one progress record per chapter per user; update status field to transition states
- Q: What should happen when the recommendation algorithm needs to compute recommendations for a user with extensive progress history (e.g., 50+ chapters tracked)? → A: Cache recommendation results for 1 hour per user; invalidate cache when profile or progress updates

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Skill Level Classification (Priority: P1)

A graduate student logs into the textbook platform after completing the 5-question profile wizard. Based on their responses (programming experience: intermediate, ROS familiarity: none, hardware access: simulation_only, learning goal: academic_research, preferred language: python), the system automatically classifies them into an appropriate skill level tier that determines how content is presented throughout their learning journey.

**Why this priority**: This is the foundation for all personalization features. Without skill level classification, the system cannot adapt content depth, recommend appropriate chapters, or adjust chatbot responses. This directly enables FR-001, FR-002, and forms the basis for all subsequent personalization features.

**Independent Test**: Can be fully tested by creating user profiles with different combinations of the 5 profile attributes, verifying the skill level classification logic returns consistent and appropriate skill levels, and confirms the classification is stored and retrievable.

**Acceptance Scenarios**:

1. **Given** a user has completed profile wizard with experience_level="beginner" and ros_familiarity="none", **When** the system classifies their skill level, **Then** they should be classified as "beginner" tier
2. **Given** a user has completed profile wizard with experience_level="advanced" and ros_familiarity="proficient", **When** the system classifies their skill level, **Then** they should be classified as "advanced" tier
3. **Given** a user has completed profile wizard with mixed attributes (experience_level="intermediate", ros_familiarity="basic"), **When** the system classifies their skill level, **Then** they should be classified as "intermediate" tier
4. **Given** a user has skipped the profile wizard, **When** the system attempts to classify their skill level, **Then** they should be assigned a default "intermediate" skill level

---

### User Story 2 - Chapter Progress Tracking (Priority: P2)

A student visits chapters as they progress through the textbook. The system tracks which chapters they have started reading, marks chapters as completed when they finish, and allows them to bookmark chapters for later review. This progress information is used to provide smarter chapter recommendations and prevent suggesting already-completed material.

**Why this priority**: Progress tracking enables recommendation relevance and prevents duplicate suggestions. It provides value independently by letting users see their learning history and resume where they left off. This is P2 because it enhances but doesn't require skill classification to function.

**Independent Test**: Can be tested by creating a test user account, recording chapter views/completions/bookmarks through API endpoints, verifying data persistence in the database, and confirming progress is retrievable across sessions.

**Acceptance Scenarios**:

1. **Given** a user opens a chapter for the first time, **When** they spend at least 10 seconds on the page, **Then** the chapter should be marked as "started" in their progress tracking
2. **Given** a user has started reading a chapter, **When** they scroll to the bottom of the chapter content, **Then** the chapter should be marked as "completed"
3. **Given** a user is viewing a chapter, **When** they click the bookmark icon, **Then** the chapter should be added to their bookmarked chapters list
4. **Given** a user has bookmarked a chapter, **When** they click the bookmark icon again, **Then** the bookmark should be removed
5. **Given** a user returns to the platform, **When** they navigate to their profile/dashboard, **Then** they should see a list of started, completed, and bookmarked chapters

---

### User Story 3 - Smart Chapter Recommendations (Priority: P3)

When a user finishes reading a chapter or visits their dashboard, the system recommends the next best chapter to read. Recommendations consider: (1) the user's skill level, (2) their learning goal, (3) chapters they've already viewed/completed, and (4) content prerequisites (e.g., don't suggest Module 3 if Module 1 isn't complete).

**Why this priority**: Recommendations add significant value but depend on both skill classification (P1) and progress tracking (P2). This is a quality-of-life improvement that guides learners through the material effectively. It's P3 because the textbook is fully usable without it.

**Independent Test**: Can be tested by creating users with different profiles and progress states, calling the recommendation API, and verifying recommended chapters match the user's skill level, avoid completed chapters, respect prerequisites, and align with learning goals.

**Acceptance Scenarios**:

1. **Given** a beginner user with no completed chapters, **When** they request chapter recommendations, **Then** the system should suggest Module 1 foundational chapters appropriate for beginners
2. **Given** an advanced user who has completed Module 1 and Module 2, **When** they request recommendations, **Then** the system should suggest Module 3 chapters and avoid suggesting already-completed chapters
3. **Given** a user with learning_goal="career_transition", **When** they request recommendations, **Then** the system should prioritize practical, hands-on chapters over purely theoretical content
4. **Given** a user with hardware_access="simulation_only", **When** they request recommendations, **Then** the system should suggest simulation-based chapters and de-prioritize hardware-dependent content
5. **Given** a user who has not completed prerequisite chapters for Module 3, **When** they request recommendations, **Then** Module 3 chapters should not be recommended until prerequisites are met

---

### User Story 4 - Adaptive Content Depth via PersonalizedSection (Priority: P2)

When a user views a chapter, the existing PersonalizedSection component already shows/hides content based on their profile. This story enhances that functionality to: (1) show code examples in the user's preferred language (Python/C++/Both), (2) display exercises matched to their experience level, and (3) only show hardware-specific content for users with appropriate hardware access.

**Why this priority**: This delivers immediate, visible personalization every time a user reads content. It enhances the existing PersonalizedSection component rather than building from scratch. It's P2 because it provides value once profiles are collected, independent of recommendations or chatbot features.

**Independent Test**: Can be tested by rendering MDX chapters with PersonalizedSection tags for users with different profiles, verifying the correct content variants are shown/hidden based on preferred_language, experience_level, and hardware_access attributes.

**Acceptance Scenarios**:

1. **Given** a user with preferred_language="python", **When** they view a chapter with both Python and C++ code examples, **Then** only Python examples should be displayed
2. **Given** a user with preferred_language="both", **When** they view a chapter with code examples, **Then** both Python and C++ examples should be displayed side-by-side
3. **Given** a user with experience_level="beginner", **When** they view a chapter with exercises, **Then** only beginner-level exercises should be shown
4. **Given** a user with hardware_access="simulation_only", **When** they view a chapter with hardware-specific instructions, **Then** hardware sections should be hidden and simulation alternatives should be shown
5. **Given** a user with hardware_access="full_robot_lab", **When** they view a chapter, **Then** all hardware-specific content should be visible

---

### User Story 5 - Profile-Aware RAG Chatbot (Priority: P3)

When an authenticated user asks a question through the RAG chatbot, the system considers their user profile to personalize the response. Specifically: (1) answers are adjusted in complexity to match the user's experience level, (2) code examples prioritize the user's preferred language, and (3) explanations reference hardware contexts appropriate to the user's hardware_access setting.

**Why this priority**: This enhances the existing RAG chatbot with profile awareness, providing smarter, more relevant answers. It's P3 because the chatbot already works for all users; this makes it better for authenticated users with profiles. It depends on the RAG pipeline already being functional.

**Independent Test**: Can be tested by submitting identical questions through the chatbot API with different user profile contexts, comparing response complexity, code language examples, and hardware references to verify they match each user's profile attributes.

**Acceptance Scenarios**:

1. **Given** a user with experience_level="beginner" asks "How do I create a ROS publisher?", **When** the chatbot generates a response, **Then** the answer should include basic explanations, avoid advanced ROS concepts, and provide step-by-step instructions
2. **Given** a user with experience_level="advanced" asks "How do I create a ROS publisher?", **When** the chatbot generates a response, **Then** the answer should be concise, assume ROS familiarity, and focus on best practices and optimization
3. **Given** a user with preferred_language="python" asks about code implementation, **When** the chatbot generates examples, **Then** code snippets should be in Python
4. **Given** a user with hardware_access="simulation_only" asks about testing a robot, **When** the chatbot generates a response, **Then** the answer should reference Gazebo simulation and avoid hardware-specific setup instructions
5. **Given** an unauthenticated user or user with incomplete profile asks a question, **When** the chatbot generates a response, **Then** the answer should use a neutral, intermediate complexity level as default

---

### Edge Cases

- **What happens when a user updates their profile after initial classification?** The system should re-classify their skill level immediately and adjust future content/recommendations, but should not affect already-completed progress.
- **How does the system handle partial profile completion?** Users who skip some questions should receive a "best-guess" skill level classification based on available profile data, defaulting to intermediate tier for missing fields.
- **What if a user has completed chapters that are now below their classified skill level?** Recommendations should respect completed chapters regardless of skill level mismatch; completed chapters are never re-recommended.
- **How does the recommendation system handle ties (multiple equally good next chapters)?** The system should return up to 3 top recommendations ranked by relevance score, and in case of exact ties, default to the lowest module number (Module 1 before Module 2).
- **What happens if the chapter dependency graph has circular dependencies?** This should be prevented at the content authoring stage; the recommendation algorithm should detect and log circular dependencies as errors but continue functioning with available non-circular paths.
- **How does adaptive content work for users with preferred_language="both"?** Show both Python and C++ examples side-by-side, with clear language labels, allowing users to compare implementations.

## Requirements *(mandatory)*

### Functional Requirements

**Skill Level Classification**
- **FR-001**: System MUST classify users into skill level tiers (beginner, intermediate, advanced) based on their profile attributes (experience_level, ros_familiarity)
- **FR-002**: System MUST store the calculated skill level classification in the database and make it retrievable via API
- **FR-003**: System MUST use a deterministic algorithm for skill level classification that produces consistent results for identical profile inputs
- **FR-004**: System MUST re-classify skill level when a user updates their profile attributes

**Chapter Progress Tracking**
- **FR-005**: System MUST track when a user starts reading a chapter (triggered by spending at least 10 seconds on the chapter page)
- **FR-006**: System MUST track when a user completes a chapter (triggered by scrolling to the bottom of the chapter content or explicit completion action)
- **FR-007**: System MUST allow users to bookmark chapters for later reference
- **FR-008**: System MUST allow users to remove bookmarks from previously bookmarked chapters
- **FR-009**: System MUST persist all progress data (started, completed, bookmarked) in the Neon Postgres database
- **FR-010**: System MUST provide API endpoints to retrieve a user's chapter progress (list of started, completed, and bookmarked chapters)
- **FR-011**: System MUST enforce a unique database constraint on (user_id, chapter_id) to prevent duplicate progress entries; subsequent actions on the same chapter must update the existing record's status and timestamps

**Chapter Recommendations**
- **FR-012**: System MUST generate personalized chapter recommendations based on: user skill level, learning goal, previously viewed/completed chapters, and content prerequisites
- **FR-013**: System MUST NOT recommend chapters the user has already completed
- **FR-014**: System MUST enforce prerequisite relationships (e.g., Module 1 chapters should be recommended before Module 3 chapters if Module 1 is incomplete)
- **FR-015**: System MUST return up to 3 top-ranked chapter recommendations per request
- **FR-016**: System MUST prioritize chapters aligned with the user's learning_goal (e.g., practical chapters for "career_transition", research-focused chapters for "academic_research")
- **FR-017**: System MUST de-prioritize chapters requiring hardware the user does not have (e.g., if hardware_access="simulation_only", de-prioritize hardware-dependent chapters)
- **FR-018**: System MUST cache recommendation results for 1 hour per user to optimize performance for users with extensive progress history
- **FR-019**: System MUST invalidate cached recommendations when user profile is updated or new chapter progress is recorded

**Adaptive Content Depth (PersonalizedSection Enhancement)**
- **FR-020**: System MUST filter code examples shown in PersonalizedSection components based on user's preferred_language (show Python only, C++ only, or both)
- **FR-021**: System MUST filter exercises shown in PersonalizedSection components based on user's experience_level (beginner, intermediate, advanced)
- **FR-022**: System MUST show/hide hardware-specific content in PersonalizedSection components based on user's hardware_access setting
- **FR-023**: System MUST support fallback behavior for users with incomplete profiles (show all content variants or use intermediate defaults)

**Profile-Aware RAG Chatbot**
- **FR-024**: System MUST include user profile context (skill level, preferred_language, hardware_access) when generating RAG chatbot responses for authenticated users
- **FR-025**: System MUST adjust answer complexity in chatbot responses based on user's skill level (simpler explanations for beginners, concise answers for advanced users)
- **FR-026**: System MUST prioritize code examples in the user's preferred_language when generating chatbot responses
- **FR-027**: System MUST reference appropriate hardware context (simulation vs. physical hardware) in chatbot responses based on user's hardware_access setting
- **FR-028**: System MUST use neutral, intermediate-level defaults for unauthenticated users or users with incomplete profiles

**Integration Requirements**
- **FR-029**: System MUST integrate with existing Better Auth session management to retrieve authenticated user IDs
- **FR-030**: System MUST read user profiles using the existing ProfileService (get_profile, update_profile methods)
- **FR-031**: System MUST NOT break existing PersonalizedSection component functionality in MDX chapters
- **FR-032**: System MUST maintain compatibility with free tier limits (Neon Postgres 0.5GB storage, OpenAI API rate limits)

### Key Entities

- **SkillLevelClassification**: Represents the computed skill level tier for a user
  - **Attributes**: user_id (UUID), skill_level (beginner/intermediate/advanced), calculated_at (timestamp), based_on_profile (JSON snapshot of profile attributes used for classification)
  - **Relationships**: One-to-one with User; updated whenever profile changes

- **ChapterProgress**: Represents a user's interaction state with a specific chapter
  - **Attributes**: id (UUID), user_id (UUID), chapter_id (string, e.g., "module-1/ros-intro"), status (started/completed), started_at (timestamp), completed_at (timestamp, nullable), is_bookmarked (boolean)
  - **Relationships**: Many-to-one with User; one-to-many with Chapter (identified by chapter_id string)
  - **Uniqueness**: Database enforces unique constraint on (user_id, chapter_id); each user can have only one progress record per chapter; status field transitions from "started" to "completed"

- **ChapterRecommendation**: Represents a recommended chapter for a user
  - **Attributes**: user_id (UUID), chapter_id (string), relevance_score (float, 0.0-1.0), recommended_at (timestamp), reason (string, explains why recommended)
  - **Relationships**: Many-to-one with User; references ChapterProgress to filter out completed chapters

- **ChapterMetadata**: Represents metadata about chapters used for recommendations
  - **Attributes**: chapter_id (string), module_number (int), title (string), difficulty_level (beginner/intermediate/advanced), prerequisites (array of chapter_id strings), requires_hardware (boolean), learning_goal_tags (array of strings: ["practical", "theoretical", "research"])
  - **Relationships**: Static metadata, referenced by ChapterRecommendation and ChapterProgress

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Skill level classification completes in under 500ms for any user profile (measured from profile retrieval to classification storage)
- **SC-002**: Personalization response time (profile retrieval + content filtering + rendering) completes in under 2 seconds end-to-end for chapter page loads
- **SC-003**: Chapter recommendation relevance score averages above 0.75 across all users (validated via user feedback on "Was this recommendation helpful?" or A/B testing)
- **SC-004**: Skill level classification accuracy exceeds 80% when validated against user self-reported skill levels or performance on chapter exercises
- **SC-005**: Adaptive content reduces time-to-understanding for beginner users by at least 20% compared to non-personalized content (measured via time spent on chapters before marking as completed or quiz performance)
- **SC-006**: Profile-aware chatbot responses achieve at least 85% user satisfaction rating from authenticated users (measured via thumbs up/down feedback on chatbot answers)
- **SC-007**: At least 60% of authenticated users complete at least one recommended chapter within 7 days of receiving the recommendation (measures recommendation quality and engagement)
- **SC-008**: System handles 100 concurrent authenticated users performing personalization operations (classification, recommendations, progress tracking) without performance degradation below SC-002 threshold

## Assumptions *(optional)*

- **Assumption 1**: Phase 4A authentication and profile collection are fully functional and deployed, including Better Auth integration, ProfileService, and ProfileWizard components
- **Assumption 2**: All 10 MDX chapters (Module 1, Module 2, Module 3) already contain PersonalizedSection tags with appropriate profile-based conditions
- **Assumption 3**: The existing RAG chatbot pipeline can accept additional context parameters (user profile) without major refactoring
- **Assumption 4**: Chapter metadata (difficulty levels, prerequisites, hardware requirements, learning goal tags) can be manually authored and stored in the database or configuration files
- **Assumption 5**: Frontend chapter pages can trigger progress tracking events (e.g., scroll tracking, time-on-page tracking) without significant performance impact
- **Assumption 6**: The free tier Neon Postgres database (0.5GB) has sufficient capacity for skill classifications, progress tracking, and chapter metadata storage
- **Assumption 7**: OpenAI API rate limits allow for profile-aware RAG responses without exceeding free tier quotas (assuming moderate usage during testing/early deployment)
- **Assumption 8**: The Docusaurus static site can fetch user-specific data (progress, recommendations) via API calls to the FastAPI backend without breaking static deployment to GitHub Pages
- **Assumption 9**: Users who skip the profile wizard receive a default "intermediate" skill level classification and see all content variants (preferred_language="both", experience_level="intermediate", hardware_access="simulation_only")

## Out of Scope *(optional)*

- Email notifications to users about new chapters or recommended content
- Social features such as sharing progress with peers or viewing other users' progress
- Gamification features like badges, leaderboards, or achievement tracking
- Offline mode or Progressive Web App (PWA) capabilities for accessing content without internet
- Admin dashboard for manually adjusting personalization rules or viewing aggregate user analytics
- A/B testing infrastructure for experimentally validating recommendation algorithms (validation will rely on user feedback and manual analysis)
- Machine learning models for adaptive skill level classification (will use rule-based classification initially)
- Multi-language support for non-English content (translation is Phase 5)
- Integration with external learning management systems (LMS) like Canvas or Moodle
- User-controlled personalization settings beyond the 5-question profile wizard (e.g., manually overriding skill level or content preferences)

## Dependencies *(optional)*

**External Dependencies:**
- Neon Postgres database must remain operational and accessible from the FastAPI backend
- OpenAI API must remain accessible for profile-aware RAG chatbot responses
- Better Auth session management must be functional for retrieving authenticated user IDs

**Internal Dependencies:**
- Phase 4A must be fully deployed, including:
  - Better Auth integration for user authentication
  - ProfileService with get_profile and update_profile methods
  - User profile database schema (user_profiles table)
  - ProfileWizard component collecting 5 profile attributes
- Existing PersonalizedSection component must be functional in all MDX chapters
- RAG chatbot pipeline (from Phase 3) must support passing additional context parameters
- Chapter metadata (difficulty, prerequisites, hardware requirements, learning goals) must be authored and stored before recommendation engine can function

**Deployment Dependencies:**
- Backend changes must be deployable to Railway without breaking existing endpoints
- Frontend changes must work with static GitHub Pages deployment (all user-specific data fetched via API calls to backend)

## Constraints *(optional)*

**Technical Constraints:**
- All new data (skill classifications, progress tracking, chapter metadata) must be stored in existing Neon Postgres database to avoid adding external services
- Personalization features must work within Neon free tier storage limit (0.5GB)
- Profile-aware RAG chatbot must respect OpenAI API rate limits to avoid exceeding free tier quotas
- Frontend must remain statically deployable to GitHub Pages; no server-side rendering or backend-dependent page generation

**Performance Constraints:**
- Personalization response time must be under 2 seconds end-to-end (from user request to personalized content displayed)
- Skill level classification must complete in under 500ms
- Chapter recommendation API must return results in under 1 second (can be served from cache if available)
- Progress tracking updates (marking chapters as started/completed/bookmarked) must complete in under 300ms
- Recommendation cache must use 1-hour TTL (time-to-live) per user; cache must be invalidated immediately upon profile or progress updates to ensure recommendation freshness

**Design Constraints:**
- Must use existing ProfileService interface; cannot modify existing profile models or break compatibility with Phase 4A
- Must enhance existing PersonalizedSection component without breaking current MDX chapters
- Must integrate with existing Better Auth session management; cannot introduce alternative authentication mechanisms

**Deployment Constraints:**
- Backend changes must be compatible with Railway deployment environment (Python 3.11+, FastAPI, asyncpg)
- Frontend changes must not require server-side rendering or break Docusaurus static build process
- Database schema changes must be applied via migrations compatible with Neon Postgres serverless architecture

## Notes *(optional)*

**Implementation Priorities:**
Given the P1/P2/P3 prioritization in User Stories, the recommended implementation order is:
1. **Phase 1 (P1)**: Skill Level Classification - Build the foundational classification algorithm and database table
2. **Phase 2 (P2)**: Chapter Progress Tracking + Adaptive Content Depth - These can be implemented in parallel as they don't depend on each other
3. **Phase 3 (P3)**: Chapter Recommendations + Profile-Aware RAG Chatbot - Both depend on P1 and P2 being complete

**Recommendation Caching Strategy:**
To optimize performance for users with extensive progress history (50+ tracked chapters), the recommendation system implements caching:
- **Cache Layer**: Use Redis or in-memory Python dict (with TTL support) for storing computed recommendations per user
- **Cache Key**: `recommendations:{user_id}`
- **TTL**: 1 hour (3600 seconds)
- **Invalidation**: Clear cache entry when user profile is updated (ProfileService.update_profile) or new chapter progress is recorded (ChapterProgressService.update_progress)
- **Implementation**: FastAPI middleware or service-level decorator to check cache before computing recommendations; cache miss triggers full computation and cache write

**Content Authoring Requirements:**
For the recommendation engine to function, chapter metadata must be manually authored for all 10 existing chapters. This includes:
- difficulty_level (beginner/intermediate/advanced)
- prerequisites (array of chapter_ids that should be completed first)
- requires_hardware (boolean flag)
- learning_goal_tags (practical, theoretical, research)

This metadata can be stored in a JSON configuration file or database table. A separate task should be created for authoring this metadata.

**Fallback Behavior:**
For users with incomplete profiles or who skipped the profile wizard:
- Skill level defaults to "intermediate"
- Preferred language defaults to "both" (show all code examples)
- Hardware access defaults to "simulation_only" (safest assumption)
- Learning goal defaults to "academic_research" (most common for graduate students)

**Monitoring and Iteration:**
After deployment, the following metrics should be monitored to validate success criteria and identify areas for improvement:
- Skill level classification distribution (ensure not all users are classified as intermediate)
- Recommendation click-through rates (percentage of recommended chapters actually visited)
- Chatbot response satisfaction ratings (thumbs up/down feedback)
- Personalization API response times (ensure under 2s SLA)

**Potential Future Enhancements (Post-MVP):**
- Machine learning-based skill level classification using user interaction patterns
- Dynamic difficulty adjustment based on chapter completion times and quiz performance
- Collaborative filtering for recommendations (suggest chapters popular among similar users)
- User-controlled personalization overrides (manually set skill level or content preferences)
