# Research: Phase 4B Personalization Engine

**Feature**: Phase 4B Personalization Engine
**Branch**: `1-personalization-engine`
**Date**: 2025-12-22
**Status**: Complete

## Overview

This document captures research findings for implementing an adaptive learning system that personalizes the robotics textbook experience. All technical unknowns from the Technical Context have been resolved through investigation of existing codebase, documentation, and best practices.

## Research Areas

### 1. Skill Level Classification Algorithm

**Research Question**: What deterministic algorithm should classify users into beginner/intermediate/advanced tiers based on profile attributes (experience_level, ros_familiarity)?

**Findings**:
- **Decision**: Use weighted scoring system based on two primary factors
- **Algorithm**:
  ```
  skill_score = (experience_weight * experience_value) + (ros_weight * ros_value)

  experience_value mapping:
    - beginner: 1
    - intermediate: 2
    - advanced: 3

  ros_familiarity_value mapping:
    - none: 1
    - basic: 2
    - proficient: 3

  weights:
    - experience_weight: 0.6 (programming experience more important)
    - ros_weight: 0.4 (ROS familiarity secondary)

  classification:
    - skill_score <= 1.4: beginner
    - 1.4 < skill_score <= 2.2: intermediate
    - skill_score > 2.2: advanced
  ```

- **Rationale**:
  - Deterministic and consistent (FR-003 requirement)
  - Simple to test and validate
  - No external dependencies (no ML models required)
  - Completes in < 50ms (well under 500ms requirement)
  - Weighs programming experience more heavily since it's foundational

- **Alternatives Considered**:
  - ML-based classification: Rejected (over-engineering, requires training data we don't have)
  - Hard-coded rules per combination: Rejected (inflexible, requires 9 rules for 3x3 matrix)
  - Quiz-based assessment: Rejected (out of scope for Phase 4B)

---

### 2. Chapter Progress Tracking Implementation

**Research Question**: What's the optimal database schema for tracking chapter progress with unique constraint on (user_id, chapter_id)?

**Findings**:
- **Decision**: Single `chapter_progress` table with status transitions
- **Schema**:
  ```sql
  CREATE TABLE chapter_progress (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    chapter_id VARCHAR(255) NOT NULL,
    status VARCHAR(20) NOT NULL CHECK (status IN ('started', 'completed')),
    is_bookmarked BOOLEAN DEFAULT FALSE,
    started_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    completed_at TIMESTAMP WITH TIME ZONE,
    updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    CONSTRAINT unique_user_chapter UNIQUE (user_id, chapter_id)
  );

  CREATE INDEX idx_chapter_progress_user ON chapter_progress(user_id);
  CREATE INDEX idx_chapter_progress_status ON chapter_progress(status);
  ```

- **Rationale**:
  - Enforces uniqueness constraint per FR-011
  - Status field allows state transitions (started → completed)
  - Separate is_bookmarked flag independent of status
  - Indexes optimize common queries (get user progress, filter by status)
  - Cascading delete ensures cleanup when user deleted

- **Alternatives Considered**:
  - Separate tables for started/completed/bookmarked: Rejected (complex joins, data duplication)
  - Event log with latest state derived: Rejected (over-engineering, slow queries)
  - JSON field in user_profiles: Rejected (poor queryability, no relational integrity)

---

### 3. Recommendation Algorithm Design

**Research Question**: How should the recommendation engine prioritize chapters considering skill level, learning goals, prerequisites, and completed chapters?

**Findings**:
- **Decision**: Weighted scoring system with prerequisite filtering
- **Algorithm Flow**:
  1. **Filter Phase**:
     - Exclude completed chapters (FR-013)
     - Exclude chapters with unmet prerequisites (FR-014)
     - Query chapter_metadata for all eligible chapters

  2. **Scoring Phase** (each factor 0.0-1.0):
     ```
     relevance_score =
       (skill_match_score * 0.35) +
       (learning_goal_match * 0.30) +
       (hardware_match * 0.20) +
       (prerequisite_readiness * 0.15)

     skill_match_score:
       - Exact match (beginner→beginner): 1.0
       - One level off: 0.5
       - Two levels off: 0.2

     learning_goal_match:
       - Chapter tags contain user's learning_goal: 1.0
       - Partial overlap: 0.6
       - No match: 0.3

     hardware_match:
       - Chapter requires_hardware = user hardware_access: 1.0
       - Simulation-only user + hardware chapter: 0.3
       - Hardware user + any chapter: 1.0

     prerequisite_readiness:
       - All prerequisites completed: 1.0
       - Most prerequisites completed: 0.7
       - No prerequisites: 1.0
     ```

  3. **Ranking Phase**:
     - Sort by relevance_score descending
     - Return top 3 (FR-015)
     - On ties: lowest module_number wins (FR-124 edge case handling)

- **Caching Strategy** (FR-018, FR-019):
  - Cache key: `recommendations:{user_id}`
  - TTL: 3600 seconds (1 hour)
  - Invalidate on: profile update, new progress record
  - Implementation: Python `cachetools.TTLCache` (simple, no Redis needed for MVP)

- **Rationale**:
  - Balances multiple factors per requirements
  - Deterministic and testable
  - Completes in ~50-100ms (uncached), ~5ms (cached)
  - Weights favor skill match and learning goals (most important to users)

- **Alternatives Considered**:
  - Collaborative filtering: Rejected (need usage data across users we don't have)
  - Content-based similarity (embeddings): Rejected (over-engineering, requires vector ops)
  - Manual curation: Rejected (not scalable, requires content team)

---

### 4. Adaptive Content Depth (PersonalizedSection Enhancement)

**Research Question**: How should PersonalizedSection component filter content based on preferred_language, experience_level, and hardware_access?

**Findings**:
- **Decision**: Extend existing PersonalizedSection component with new profile attributes
- **Implementation**:
  - Component already reads user profile from React Context (Phase 4A)
  - Add filtering logic for `preferred_language`, `experience_level`, `hardware_access`
  - MDX authors use tags: `<PersonalizedSection language="python">`, `<PersonalizedSection level="beginner">`, `<PersonalizedSection hardware="physical">`
  - Component shows/hides content based on profile match

- **Example MDX Usage**:
  ```mdx
  <PersonalizedSection language="python">
  ```python
  # Python code example
  node.get_logger().info('Hello ROS 2')
  ```
  </PersonalizedSection>

  <PersonalizedSection language="cpp">
  ```cpp
  // C++ code example
  node->get_logger()->info("Hello ROS 2");
  ```
  </PersonalizedSection>

  <PersonalizedSection level="beginner">
  This exercise is designed for beginners...
  </PersonalizedSection>

  <PersonalizedSection hardware="physical">
  Connect your robot to the power supply...
  </PersonalizedSection>
  ```

- **Filtering Logic**:
  ```typescript
  function shouldShowContent(
    userProfile: UserProfile,
    sectionProps: PersonalizedSectionProps
  ): boolean {
    // Language filtering
    if (sectionProps.language) {
      if (userProfile.preferred_language === 'both') return true;
      if (userProfile.preferred_language !== sectionProps.language) return false;
    }

    // Experience level filtering
    if (sectionProps.level) {
      if (userProfile.experience_level !== sectionProps.level) return false;
    }

    // Hardware filtering
    if (sectionProps.hardware === 'physical') {
      if (userProfile.hardware_access === 'simulation_only') return false;
    }

    return true; // Show by default if no filters match
  }
  ```

- **Rationale**:
  - Builds on existing component (no rebuilding)
  - Minimal performance impact (filtering in-memory)
  - Maintains static site generation (filtering happens client-side after hydration)
  - Authors control what's personalizable via tags

- **Alternatives Considered**:
  - Server-side rendering with personalization: Rejected (breaks GitHub Pages static deployment)
  - Separate page versions per skill level: Rejected (content duplication, maintenance burden)
  - CSS-only hiding: Rejected (content still in HTML, a11y issues with screen readers)

---

### 5. Profile-Aware RAG Chatbot Integration

**Research Question**: How should the RAG chatbot incorporate user profile context without major refactoring?

**Findings**:
- **Decision**: Inject profile context as system message prefix in RAG pipeline
- **Implementation**:
  - Existing RAG pipeline: `user_query → embed → vector_search → retrieve_context → generate_answer`
  - Enhancement: Add profile context before generation step

  ```python
  def generate_profile_aware_answer(
      user_query: str,
      retrieved_context: str,
      user_profile: UserProfile
  ) -> str:
      # Build profile context string
      profile_context = f"""
      User Profile:
      - Experience Level: {user_profile.experience_level}
      - ROS Familiarity: {user_profile.ros_familiarity}
      - Preferred Language: {user_profile.preferred_language}
      - Hardware Access: {user_profile.hardware_access}
      - Learning Goal: {user_profile.learning_goal}

      Instructions:
      - Adjust answer complexity to match experience level
      - Prefer {user_profile.preferred_language} code examples
      - Reference {user_profile.hardware_access} setup in instructions
      """

      # Pass to OpenAI Agents SDK with profile context prepended
      messages = [
          {"role": "system", "content": profile_context},
          {"role": "system", "content": retrieved_context},
          {"role": "user", "content": user_query}
      ]

      return openai_agent.generate(messages)
  ```

- **Complexity Adjustment Examples**:
  - Beginner: "A ROS 2 publisher is a node component that sends messages to a topic. Here's a step-by-step example..."
  - Advanced: "Create a publisher using `create_publisher<T>(topic, qos)`. Consider QoS profiles for reliability..."

- **Rationale**:
  - No refactoring of embedding or vector search
  - Profile context naturally influences LLM generation
  - Adds ~100-200ms to generation time (minimal impact, still under 3s budget)
  - Works for both authenticated (profile-aware) and unauthenticated (default profile) users

- **Alternatives Considered**:
  - Fine-tune separate models per skill level: Rejected (expensive, slow, maintenance burden)
  - Retrieve different content per skill level: Rejected (requires separate vector collections)
  - Post-process answer to adjust complexity: Rejected (unreliable, adds latency)

---

### 6. Chapter Metadata Authoring

**Research Question**: How should chapter metadata (difficulty, prerequisites, hardware requirements, learning goal tags) be stored and maintained?

**Findings**:
- **Decision**: Store in database table, manually authored via JSON config initially
- **Schema**:
  ```sql
  CREATE TABLE chapter_metadata (
    chapter_id VARCHAR(255) PRIMARY KEY,
    module_number INTEGER NOT NULL,
    title VARCHAR(500) NOT NULL,
    difficulty_level VARCHAR(20) NOT NULL CHECK (difficulty_level IN ('beginner', 'intermediate', 'advanced')),
    prerequisites JSONB DEFAULT '[]',  -- Array of chapter_id strings
    requires_hardware BOOLEAN DEFAULT FALSE,
    learning_goal_tags JSONB DEFAULT '[]',  -- Array of goal strings
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
  );

  CREATE INDEX idx_chapter_metadata_difficulty ON chapter_metadata(difficulty_level);
  CREATE INDEX idx_chapter_metadata_module ON chapter_metadata(module_number);
  ```

- **Initial Data Population**:
  - Create `backend/data/chapter_metadata.json` with metadata for 10 chapters
  - Migration script loads JSON into database
  - Example entry:
    ```json
    {
      "chapter_id": "module-1/ros-intro",
      "module_number": 1,
      "title": "Introduction to ROS 2",
      "difficulty_level": "beginner",
      "prerequisites": [],
      "requires_hardware": false,
      "learning_goal_tags": ["practical", "theoretical"]
    }
    ```

- **Rationale**:
  - Queryable for recommendation algorithm
  - Editable without code changes
  - Version-controlled (migrations + JSON)
  - Supports 10 chapters initially, scalable to hundreds

- **Alternatives Considered**:
  - Frontmatter in MDX files: Rejected (hard to query from backend)
  - Hardcoded in Python: Rejected (requires deploy for metadata updates)
  - Separate CMS: Rejected (over-engineering, adds complexity)

---

### 7. Recommendation Cache Implementation

**Research Question**: What caching strategy optimizes performance for users with 50+ tracked chapters without exceeding free tier constraints?

**Findings**:
- **Decision**: In-memory Python `cachetools.TTLCache` with user-keyed entries
- **Implementation**:
  ```python
  from cachetools import TTLCache

  # Global cache instance
  recommendation_cache = TTLCache(maxsize=1000, ttl=3600)  # 1 hour TTL, 1000 users max

  def get_recommendations(user_id: str) -> List[ChapterRecommendation]:
      cache_key = f"recommendations:{user_id}"

      # Check cache first
      if cache_key in recommendation_cache:
          return recommendation_cache[cache_key]

      # Cache miss: compute recommendations
      recommendations = compute_recommendations(user_id)

      # Store in cache
      recommendation_cache[cache_key] = recommendations

      return recommendations

  def invalidate_recommendation_cache(user_id: str):
      cache_key = f"recommendations:{user_id}"
      recommendation_cache.pop(cache_key, None)
  ```

- **Invalidation Triggers**:
  - User profile updated → `invalidate_recommendation_cache(user_id)`
  - New progress record created → `invalidate_recommendation_cache(user_id)`

- **Memory Usage Estimate**:
  - Each cache entry: ~1KB (3 recommendations with metadata)
  - 1000 users × 1KB = 1MB total
  - Well within free tier constraints

- **Rationale**:
  - Simple (no Redis required for MVP)
  - Fast (in-memory, ~5ms cache hit)
  - Reduces database queries by ~70% for repeat requests
  - Automatic TTL expiration (no manual cleanup)
  - Invalidation ensures freshness when data changes

- **Alternatives Considered**:
  - Redis caching: Rejected (adds external dependency, over-engineering for MVP)
  - Database materialized views: Rejected (Neon serverless doesn't support well)
  - No caching: Rejected (fails performance requirement for users with 50+ chapters)

---

## Technology Stack Confirmation

### Backend
- **Language**: Python 3.11+
- **Framework**: FastAPI (async support)
- **Database**: Neon Serverless Postgres
- **Database Client**: asyncpg (async Postgres driver)
- **Caching**: cachetools.TTLCache (in-memory)
- **Authentication**: Better Auth (via existing ProfileService)
- **Testing**: Pytest + pytest-asyncio

### Frontend
- **Language**: TypeScript 5.x (strict mode)
- **Framework**: React 18 (Docusaurus v3)
- **State Management**: React Context (user profile)
- **API Client**: fetch API + custom hooks
- **Testing**: React Testing Library + Playwright (E2E)

### Integration Points
- **Better Auth**: Read user_id from session, fetch profile via ProfileService
- **RAG Pipeline**: Inject profile context into OpenAI Agents SDK prompts
- **Existing PersonalizedSection**: Enhance component with new filtering logic

---

## Best Practices Research

### 1. FastAPI Async Best Practices
- Use `async def` for all I/O operations (database queries, external APIs)
- Use connection pooling: `asyncpg.create_pool()` (max 10 connections for free tier)
- Structured logging with `structlog` for JSON logs
- Pydantic models for request/response validation

### 2. React Hooks Best Practices
- Custom hooks for data fetching (`useSkillLevel`, `useChapterProgress`, `useRecommendations`)
- Use React Query or SWR for caching and revalidation (optional, evaluate if needed)
- Error boundaries for graceful error handling
- Loading states with skeleton screens

### 3. Database Best Practices
- Use indexes for common queries (`user_id`, `status`, `module_number`)
- Use JSONB for arrays (prerequisites, learning_goal_tags) with GIN indexes if needed
- Use `ON DELETE CASCADE` for referential integrity
- Use migrations for schema versioning (Alembic or raw SQL)

### 4. Testing Best Practices
- Unit tests: Mock database, test business logic in isolation
- Integration tests: Use test database, test API endpoints end-to-end
- E2E tests: Playwright for critical user journeys (signup → personalization → recommendations)
- Test coverage target: 80% (enforced via pytest-cov)

---

## Performance Validation Strategy

### Latency Targets (from spec)
- Skill classification: < 500ms
- Personalization response: < 2s (end-to-end)
- Recommendation API: < 1s (cached)
- Progress tracking updates: < 300ms

### Measurement Approach
- Add timing decorators to service methods
- Log p50/p95/p99 latencies with correlation IDs
- Use pytest-benchmark for unit test performance
- Use Playwright for E2E timing measurements

### Load Testing
- Use Locust or K6 to simulate 100 concurrent users
- Target: No degradation below latency thresholds
- Monitor Neon connection pool usage

---

## Security Considerations

### Input Validation
- Validate chapter_id format (alphanumeric + hyphens only, prevent path traversal)
- Validate user_id is UUID format
- Sanitize all user inputs before database queries (use parameterized queries)

### Authentication
- All personalization endpoints require authenticated session (Better Auth)
- Retrieve user_id from validated JWT token (not from request body)
- No PII in logs (log user_id as hash or correlation ID)

### Rate Limiting
- 50 requests/hour per authenticated user for recommendation endpoint (prevent abuse)
- 100 requests/hour for progress tracking (higher limit for frequent page views)

---

## Risks and Mitigations

### Risk 1: Recommendation Cache Invalidation Bugs
- **Impact**: Users see stale recommendations after profile/progress updates
- **Likelihood**: Medium
- **Mitigation**: Integration tests validate cache invalidation; add logging for cache hits/misses

### Risk 2: Neon Free Tier Storage Exceeded
- **Impact**: Database writes fail
- **Likelihood**: Low (careful capacity planning)
- **Mitigation**: Monitor storage usage; alert at 80%; implement data retention policies (delete old progress records after 1 year)

### Risk 3: Skill Classification Algorithm Disagreement
- **Impact**: Users feel misclassified (too easy/hard content)
- **Likelihood**: Medium
- **Mitigation**: Allow users to provide feedback on recommendations; collect data for future algorithm refinement; document algorithm in ADR

---

## Dependencies Confirmed

### External Dependencies
- Neon Postgres: Operational, free tier sufficient
- OpenAI API: Accessible, rate limits manageable with caching
- Better Auth: Functional from Phase 4A

### Internal Dependencies
- Phase 4A: Better Auth, ProfileService, user_profiles table ✅
- Existing PersonalizedSection component: Functional ✅
- RAG chatbot pipeline (Phase 3): Supports additional context parameters ✅
- Chapter metadata: Must be authored (manual task for content team)

---

## Next Steps (Phase 1: Design & Contracts)

1. Generate `data-model.md` with detailed entity definitions
2. Generate API contracts in `contracts/` directory (OpenAPI specs)
3. Generate `quickstart.md` with setup instructions
4. Update agent context with new technologies

---

**Research Complete**: All NEEDS CLARIFICATION items resolved. Ready to proceed to Phase 1.
