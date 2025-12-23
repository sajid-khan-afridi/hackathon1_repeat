# Data Model: Phase 4B Personalization Engine

**Feature**: Phase 4B Personalization Engine
**Branch**: `1-personalization-engine`
**Date**: 2025-12-22
**Status**: Complete

## Overview

This document defines all data entities for the personalization engine, including database schema, relationships, validation rules, and state transitions. All entities are stored in Neon Serverless Postgres.

---

## Entities

### 1. SkillLevelClassification

**Purpose**: Stores computed skill level tier for a user based on their profile attributes.

**Attributes**:
- `id` (UUID, Primary Key): Unique identifier for the classification record
- `user_id` (UUID, Foreign Key → users.id, NOT NULL, UNIQUE): User this classification belongs to
- `skill_level` (ENUM, NOT NULL): Computed skill tier
  - Allowed values: `'beginner'`, `'intermediate'`, `'advanced'`
- `calculated_at` (TIMESTAMP WITH TIME ZONE, NOT NULL, DEFAULT NOW()): When classification was computed
- `updated_at` (TIMESTAMP WITH TIME ZONE, NOT NULL, DEFAULT NOW()): Last update timestamp
- `based_on_profile` (JSONB, NOT NULL): Snapshot of profile attributes used for classification
  - Format: `{"experience_level": "intermediate", "ros_familiarity": "basic", "hardware_access": "simulation_only", "learning_goal": "academic_research", "preferred_language": "python"}`

**Relationships**:
- **One-to-One with User**: Each user has exactly one skill level classification
- **References**: `users` table (from Phase 4A)

**Validation Rules**:
- `skill_level` must be one of: `'beginner'`, `'intermediate'`, `'advanced'`
- `user_id` must exist in `users` table
- `based_on_profile` must contain required keys: `experience_level`, `ros_familiarity`
- Unique constraint on `user_id` (only one classification per user)

**State Transitions**:
- **Create**: When user completes profile wizard or profile service is first accessed
- **Update**: When user updates their profile attributes (experience_level or ros_familiarity change)
- **Update Trigger**: Recalculate skill_level using classification algorithm; store new based_on_profile snapshot

**Database Schema**:
```sql
CREATE TABLE skill_level_classifications (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID NOT NULL UNIQUE REFERENCES users(id) ON DELETE CASCADE,
  skill_level VARCHAR(20) NOT NULL CHECK (skill_level IN ('beginner', 'intermediate', 'advanced')),
  calculated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  based_on_profile JSONB NOT NULL
);

CREATE INDEX idx_skill_level_user ON skill_level_classifications(user_id);
CREATE INDEX idx_skill_level_tier ON skill_level_classifications(skill_level);
```

**Example Data**:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "user_id": "123e4567-e89b-12d3-a456-426614174000",
  "skill_level": "intermediate",
  "calculated_at": "2025-12-22T10:30:00Z",
  "updated_at": "2025-12-22T10:30:00Z",
  "based_on_profile": {
    "experience_level": "intermediate",
    "ros_familiarity": "basic",
    "hardware_access": "simulation_only",
    "learning_goal": "academic_research",
    "preferred_language": "python"
  }
}
```

---

### 2. ChapterProgress

**Purpose**: Tracks a user's interaction state with a specific chapter (started, completed, bookmarked).

**Attributes**:
- `id` (UUID, Primary Key): Unique identifier for the progress record
- `user_id` (UUID, Foreign Key → users.id, NOT NULL): User this progress belongs to
- `chapter_id` (VARCHAR(255), NOT NULL): Identifier for the chapter (e.g., `"module-1/ros-intro"`)
- `status` (ENUM, NOT NULL): Current progress status
  - Allowed values: `'started'`, `'completed'`
- `is_bookmarked` (BOOLEAN, NOT NULL, DEFAULT FALSE): Whether user has bookmarked this chapter
- `started_at` (TIMESTAMP WITH TIME ZONE, NOT NULL, DEFAULT NOW()): When user first opened the chapter
- `completed_at` (TIMESTAMP WITH TIME ZONE, NULLABLE): When user marked chapter as completed
- `updated_at` (TIMESTAMP WITH TIME ZONE, NOT NULL, DEFAULT NOW()): Last update timestamp

**Relationships**:
- **Many-to-One with User**: Each user can have many progress records
- **Implicitly References**: Chapter content (identified by `chapter_id` string, no foreign key to chapter table since chapters are static MDX files)

**Validation Rules**:
- Unique constraint on `(user_id, chapter_id)`: Only one progress record per user per chapter (FR-011)
- `status` must be one of: `'started'`, `'completed'`
- `completed_at` must be NULL if `status = 'started'`
- `completed_at` must be NOT NULL if `status = 'completed'`
- `completed_at` must be >= `started_at` (if set)
- `chapter_id` format: alphanumeric + hyphens + slashes only (e.g., `module-1/ros-intro`)

**State Transitions**:
1. **Create (Started)**: User spends 10+ seconds on chapter page → Create record with `status = 'started'`, `started_at = NOW()`
2. **Update (Completed)**: User scrolls to bottom or explicitly marks complete → Update existing record: `status = 'completed'`, `completed_at = NOW()`
3. **Update (Bookmark Toggle)**: User clicks bookmark icon → Toggle `is_bookmarked`, update `updated_at`

**Database Schema**:
```sql
CREATE TABLE chapter_progress (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
  chapter_id VARCHAR(255) NOT NULL,
  status VARCHAR(20) NOT NULL CHECK (status IN ('started', 'completed')),
  is_bookmarked BOOLEAN NOT NULL DEFAULT FALSE,
  started_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  completed_at TIMESTAMP WITH TIME ZONE,
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  CONSTRAINT unique_user_chapter UNIQUE (user_id, chapter_id),
  CONSTRAINT completed_at_valid CHECK (
    (status = 'started' AND completed_at IS NULL) OR
    (status = 'completed' AND completed_at IS NOT NULL)
  )
);

CREATE INDEX idx_chapter_progress_user ON chapter_progress(user_id);
CREATE INDEX idx_chapter_progress_status ON chapter_progress(status);
CREATE INDEX idx_chapter_progress_chapter ON chapter_progress(chapter_id);
```

**Example Data**:
```json
{
  "id": "650e8400-e29b-41d4-a716-446655440001",
  "user_id": "123e4567-e89b-12d3-a456-426614174000",
  "chapter_id": "module-1/ros-intro",
  "status": "completed",
  "is_bookmarked": true,
  "started_at": "2025-12-20T14:00:00Z",
  "completed_at": "2025-12-20T15:30:00Z",
  "updated_at": "2025-12-21T09:00:00Z"
}
```

---

### 3. ChapterRecommendation

**Purpose**: Represents a recommended chapter for a user (ephemeral, computed on-demand, cached).

**Attributes**:
- `user_id` (UUID): User this recommendation is for
- `chapter_id` (VARCHAR(255)): Identifier for the recommended chapter
- `relevance_score` (FLOAT, 0.0-1.0): Computed relevance score from recommendation algorithm
- `recommended_at` (TIMESTAMP WITH TIME ZONE): When recommendation was generated
- `reason` (TEXT): Human-readable explanation of why this chapter was recommended
  - Example: `"Matches your beginner skill level and academic research goal"`

**Relationships**:
- **Many-to-One with User**: Each user can have many recommendations (typically 3 at a time)
- **References ChapterMetadata**: Implicitly via `chapter_id`
- **References ChapterProgress**: Algorithm filters out completed chapters

**Validation Rules**:
- `relevance_score` must be between 0.0 and 1.0
- `chapter_id` must exist in `chapter_metadata` table
- Must not recommend chapters with `status = 'completed'` in user's `chapter_progress`

**State Transitions**:
- **Compute**: User requests recommendations → Run algorithm → Generate top 3 recommendations → Cache for 1 hour
- **Invalidate**: User updates profile or records progress → Clear cache → Next request recomputes

**Storage**:
- **NOT stored in database** (computed on-demand, cached in-memory)
- **Cache Location**: Python `cachetools.TTLCache` with key `recommendations:{user_id}`
- **Cache TTL**: 3600 seconds (1 hour)

**Example Data** (in-memory cache entry):
```json
[
  {
    "user_id": "123e4567-e89b-12d3-a456-426614174000",
    "chapter_id": "module-2/ros-publishers",
    "relevance_score": 0.92,
    "recommended_at": "2025-12-22T10:30:00Z",
    "reason": "Matches your intermediate skill level and completes Module 1 prerequisites"
  },
  {
    "user_id": "123e4567-e89b-12d3-a456-426614174000",
    "chapter_id": "module-2/ros-subscribers",
    "relevance_score": 0.88,
    "recommended_at": "2025-12-22T10:30:00Z",
    "reason": "Practical content aligns with your career transition goal"
  },
  {
    "user_id": "123e4567-e89b-12d3-a456-426614174000",
    "chapter_id": "module-1/gazebo-simulation",
    "relevance_score": 0.85,
    "recommended_at": "2025-12-22T10:30:00Z",
    "reason": "Simulation-focused chapter matches your hardware access setting"
  }
]
```

---

### 4. ChapterMetadata

**Purpose**: Stores metadata about chapters used for recommendations (difficulty, prerequisites, hardware requirements, learning goal tags).

**Attributes**:
- `chapter_id` (VARCHAR(255), Primary Key): Unique identifier for the chapter (matches chapter_id in ChapterProgress)
- `module_number` (INTEGER, NOT NULL): Module number (1, 2, 3, etc.) for prerequisite ordering
- `title` (VARCHAR(500), NOT NULL): Human-readable chapter title
- `difficulty_level` (ENUM, NOT NULL): Difficulty classification
  - Allowed values: `'beginner'`, `'intermediate'`, `'advanced'`
- `prerequisites` (JSONB, NOT NULL, DEFAULT '[]'): Array of chapter_id strings that must be completed first
  - Example: `["module-1/ros-intro", "module-1/linux-basics"]`
- `requires_hardware` (BOOLEAN, NOT NULL, DEFAULT FALSE): Whether chapter requires physical robot hardware
- `learning_goal_tags` (JSONB, NOT NULL, DEFAULT '[]'): Array of learning goal tags
  - Allowed values: `"practical"`, `"theoretical"`, `"research"`
  - Example: `["practical", "theoretical"]`
- `created_at` (TIMESTAMP WITH TIME ZONE, NOT NULL, DEFAULT NOW()): When metadata was created
- `updated_at` (TIMESTAMP WITH TIME ZONE, NOT NULL, DEFAULT NOW()): Last update timestamp

**Relationships**:
- **Referenced by ChapterProgress**: Via `chapter_id`
- **Referenced by ChapterRecommendation**: Via `chapter_id`

**Validation Rules**:
- `chapter_id` format: alphanumeric + hyphens + slashes only
- `difficulty_level` must be one of: `'beginner'`, `'intermediate'`, `'advanced'`
- `module_number` must be > 0
- `prerequisites` must be valid JSON array of strings
- `learning_goal_tags` must be valid JSON array containing only: `"practical"`, `"theoretical"`, `"research"`

**State Transitions**:
- **Create**: Content team authors new chapter → Add metadata row
- **Update**: Content team updates chapter difficulty/prerequisites → Update metadata row

**Database Schema**:
```sql
CREATE TABLE chapter_metadata (
  chapter_id VARCHAR(255) PRIMARY KEY,
  module_number INTEGER NOT NULL CHECK (module_number > 0),
  title VARCHAR(500) NOT NULL,
  difficulty_level VARCHAR(20) NOT NULL CHECK (difficulty_level IN ('beginner', 'intermediate', 'advanced')),
  prerequisites JSONB NOT NULL DEFAULT '[]',
  requires_hardware BOOLEAN NOT NULL DEFAULT FALSE,
  learning_goal_tags JSONB NOT NULL DEFAULT '[]',
  created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_chapter_metadata_difficulty ON chapter_metadata(difficulty_level);
CREATE INDEX idx_chapter_metadata_module ON chapter_metadata(module_number);
CREATE INDEX idx_chapter_metadata_hardware ON chapter_metadata(requires_hardware);
```

**Example Data**:
```json
{
  "chapter_id": "module-2/ros-publishers",
  "module_number": 2,
  "title": "Creating ROS 2 Publishers",
  "difficulty_level": "intermediate",
  "prerequisites": ["module-1/ros-intro", "module-1/python-basics"],
  "requires_hardware": false,
  "learning_goal_tags": ["practical", "theoretical"],
  "created_at": "2025-12-15T08:00:00Z",
  "updated_at": "2025-12-15T08:00:00Z"
}
```

---

## Entity Relationships Diagram

```
┌─────────────────────────────────┐
│ users (Phase 4A)                │
│ ├─ id (UUID, PK)                │
│ ├─ email                        │
│ └─ ...                          │
└────────┬────────────────────────┘
         │
         │ 1:1
         ▼
┌─────────────────────────────────┐
│ skill_level_classifications     │
│ ├─ id (UUID, PK)                │
│ ├─ user_id (FK → users.id)     │
│ ├─ skill_level (ENUM)           │
│ ├─ calculated_at                │
│ ├─ updated_at                   │
│ └─ based_on_profile (JSONB)    │
└────────┬────────────────────────┘
         │
         │ 1:N
         │
┌─────────▼───────────────────────┐
│ chapter_progress                │
│ ├─ id (UUID, PK)                │
│ ├─ user_id (FK → users.id)     │
│ ├─ chapter_id (VARCHAR)        │───┐
│ ├─ status (ENUM)               │    │
│ ├─ is_bookmarked               │    │
│ ├─ started_at                  │    │
│ ├─ completed_at                │    │
│ └─ updated_at                  │    │
│ UNIQUE (user_id, chapter_id)   │    │
└─────────────────────────────────┘    │
                                        │
                                        │ References (implicit)
                                        │
                                        │
                                        ▼
                            ┌───────────────────────────┐
                            │ chapter_metadata          │
                            │ ├─ chapter_id (PK)       │
                            │ ├─ module_number         │
                            │ ├─ title                 │
                            │ ├─ difficulty_level      │
                            │ ├─ prerequisites (JSONB) │
                            │ ├─ requires_hardware     │
                            │ ├─ learning_goal_tags    │
                            │ ├─ created_at            │
                            │ └─ updated_at            │
                            └───────────────────────────┘
                                        │
                                        │ Used by (computed, not stored)
                                        │
                                        ▼
                            ┌───────────────────────────┐
                            │ ChapterRecommendation     │
                            │ (In-Memory Cache)         │
                            │ ├─ user_id               │
                            │ ├─ chapter_id            │
                            │ ├─ relevance_score       │
                            │ ├─ recommended_at        │
                            │ └─ reason                │
                            └───────────────────────────┘
```

---

## Data Flow

### 1. Skill Level Classification Flow
```
User completes profile wizard (Phase 4A)
  ↓
ProfileService.update_profile(user_id, attributes)
  ↓
ClassificationService.classify_user(user_id)
  ↓
Fetch user_profile from database
  ↓
Apply weighted scoring algorithm:
  skill_score = (experience_weight * experience_value) + (ros_weight * ros_value)
  ↓
Determine skill_level (beginner/intermediate/advanced)
  ↓
INSERT or UPDATE skill_level_classifications table
  ↓
Return skill_level to caller
```

### 2. Chapter Progress Tracking Flow
```
User opens chapter page (frontend)
  ↓
Frontend: Start timer, wait 10 seconds
  ↓
If user still on page after 10s:
  POST /api/v1/progress/start { chapter_id: "module-1/ros-intro" }
  ↓
Backend: ProgressTrackingService.mark_started(user_id, chapter_id)
  ↓
INSERT INTO chapter_progress (user_id, chapter_id, status='started')
  ON CONFLICT (user_id, chapter_id) DO NOTHING
  ↓
Return 201 Created or 200 OK (if already exists)

---

User scrolls to bottom of chapter (frontend)
  ↓
POST /api/v1/progress/complete { chapter_id: "module-1/ros-intro" }
  ↓
Backend: ProgressTrackingService.mark_completed(user_id, chapter_id)
  ↓
UPDATE chapter_progress
  SET status='completed', completed_at=NOW(), updated_at=NOW()
  WHERE user_id=? AND chapter_id=?
  ↓
Invalidate recommendation cache for user
  ↓
Return 200 OK
```

### 3. Chapter Recommendation Flow
```
User requests recommendations (frontend)
  ↓
GET /api/v1/recommendations
  ↓
Backend: RecommendationService.get_recommendations(user_id)
  ↓
Check cache: recommendations:{user_id}
  ↓
IF cache hit:
  Return cached recommendations (5ms)
ELSE:
  ↓
  Fetch user skill_level from skill_level_classifications
  Fetch user profile (learning_goal, hardware_access)
  Fetch user progress (completed chapters)
  Fetch all chapter_metadata
  ↓
  Filter Phase:
    - Exclude completed chapters
    - Exclude chapters with unmet prerequisites
  ↓
  Scoring Phase:
    For each eligible chapter:
      skill_match_score = match_skill_level(user_skill, chapter_difficulty)
      learning_goal_match = match_tags(user_goal, chapter_tags)
      hardware_match = match_hardware(user_hardware, chapter_requires_hardware)
      prerequisite_readiness = calculate_prereq_completion(user_progress, chapter_prereqs)

      relevance_score = (skill_match * 0.35) + (goal_match * 0.30) +
                        (hardware_match * 0.20) + (prereq_readiness * 0.15)
  ↓
  Ranking Phase:
    Sort by relevance_score DESC
    Take top 3
    On ties: prefer lower module_number
  ↓
  Store in cache (TTL = 1 hour)
  ↓
  Return recommendations (50-100ms for computation)
```

### 4. Profile-Aware RAG Chatbot Flow
```
User submits question (frontend)
  ↓
POST /api/v1/chat/query { query: "How do I create a ROS publisher?" }
  ↓
Backend: Authenticate user, fetch user_profile
  ↓
IF user authenticated:
  Build profile_context string:
    "User Profile: experience_level=intermediate, preferred_language=python, ..."
ELSE:
  Use default profile_context
  ↓
Existing RAG Pipeline:
  Embed query → Search Qdrant → Retrieve top chunks
  ↓
Generate Answer:
  messages = [
    {"role": "system", "content": profile_context},
    {"role": "system", "content": retrieved_chunks},
    {"role": "user", "content": user_query}
  ]
  ↓
OpenAI Agents SDK → Generate response
  ↓
Return answer (personalized complexity + code language)
```

---

## Database Migration Script

**File**: `backend/src/migrations/005_personalization_schema.sql`

```sql
-- Migration: 005_personalization_schema
-- Description: Add tables for Phase 4B personalization engine
-- Date: 2025-12-22
-- Dependencies: Requires Phase 4A users and user_profiles tables

-- 1. Skill Level Classifications Table
CREATE TABLE IF NOT EXISTS skill_level_classifications (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID NOT NULL UNIQUE REFERENCES users(id) ON DELETE CASCADE,
  skill_level VARCHAR(20) NOT NULL CHECK (skill_level IN ('beginner', 'intermediate', 'advanced')),
  calculated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  based_on_profile JSONB NOT NULL
);

CREATE INDEX idx_skill_level_user ON skill_level_classifications(user_id);
CREATE INDEX idx_skill_level_tier ON skill_level_classifications(skill_level);

-- 2. Chapter Progress Table
CREATE TABLE IF NOT EXISTS chapter_progress (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
  chapter_id VARCHAR(255) NOT NULL,
  status VARCHAR(20) NOT NULL CHECK (status IN ('started', 'completed')),
  is_bookmarked BOOLEAN NOT NULL DEFAULT FALSE,
  started_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  completed_at TIMESTAMP WITH TIME ZONE,
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  CONSTRAINT unique_user_chapter UNIQUE (user_id, chapter_id),
  CONSTRAINT completed_at_valid CHECK (
    (status = 'started' AND completed_at IS NULL) OR
    (status = 'completed' AND completed_at IS NOT NULL)
  )
);

CREATE INDEX idx_chapter_progress_user ON chapter_progress(user_id);
CREATE INDEX idx_chapter_progress_status ON chapter_progress(status);
CREATE INDEX idx_chapter_progress_chapter ON chapter_progress(chapter_id);
CREATE INDEX idx_chapter_progress_bookmarked ON chapter_progress(is_bookmarked) WHERE is_bookmarked = TRUE;

-- 3. Chapter Metadata Table
CREATE TABLE IF NOT EXISTS chapter_metadata (
  chapter_id VARCHAR(255) PRIMARY KEY,
  module_number INTEGER NOT NULL CHECK (module_number > 0),
  title VARCHAR(500) NOT NULL,
  difficulty_level VARCHAR(20) NOT NULL CHECK (difficulty_level IN ('beginner', 'intermediate', 'advanced')),
  prerequisites JSONB NOT NULL DEFAULT '[]',
  requires_hardware BOOLEAN NOT NULL DEFAULT FALSE,
  learning_goal_tags JSONB NOT NULL DEFAULT '[]',
  created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_chapter_metadata_difficulty ON chapter_metadata(difficulty_level);
CREATE INDEX idx_chapter_metadata_module ON chapter_metadata(module_number);
CREATE INDEX idx_chapter_metadata_hardware ON chapter_metadata(requires_hardware);

-- 4. Populate Initial Chapter Metadata (10 sample chapters)
-- This data should match the MDX chapters created in Phase 2
INSERT INTO chapter_metadata (chapter_id, module_number, title, difficulty_level, prerequisites, requires_hardware, learning_goal_tags) VALUES
  ('module-1/ros-intro', 1, 'Introduction to ROS 2', 'beginner', '[]', FALSE, '["theoretical", "practical"]'),
  ('module-1/linux-basics', 1, 'Linux Basics for Robotics', 'beginner', '[]', FALSE, '["practical"]'),
  ('module-1/python-basics', 1, 'Python Programming for ROS', 'beginner', '[]', FALSE, '["practical"]'),
  ('module-1/gazebo-simulation', 1, 'Gazebo Simulation Setup', 'beginner', '["module-1/linux-basics"]', FALSE, '["practical"]'),
  ('module-2/ros-publishers', 2, 'Creating ROS 2 Publishers', 'intermediate', '["module-1/ros-intro", "module-1/python-basics"]', FALSE, '["practical", "theoretical"]'),
  ('module-2/ros-subscribers', 2, 'Creating ROS 2 Subscribers', 'intermediate', '["module-1/ros-intro", "module-1/python-basics"]', FALSE, '["practical", "theoretical"]'),
  ('module-2/ros-services', 2, 'ROS 2 Services and Clients', 'intermediate', '["module-2/ros-publishers", "module-2/ros-subscribers"]', FALSE, '["practical", "theoretical"]'),
  ('module-3/advanced-control', 3, 'Advanced Robot Control', 'advanced', '["module-2/ros-publishers", "module-2/ros-subscribers"]', TRUE, '["practical", "research"]'),
  ('module-3/inverse-kinematics', 3, 'Inverse Kinematics for Manipulators', 'advanced', '["module-2/ros-services"]', TRUE, '["theoretical", "research"]'),
  ('module-3/isaac-integration', 3, 'NVIDIA Isaac Sim Integration', 'advanced', '["module-1/gazebo-simulation", "module-2/ros-publishers"]', FALSE, '["practical", "research"]')
ON CONFLICT (chapter_id) DO NOTHING;

-- Migration complete
```

---

## Summary

This data model supports all Phase 4B functional requirements:
- **FR-001 to FR-004**: SkillLevelClassification entity with deterministic algorithm
- **FR-005 to FR-011**: ChapterProgress entity with unique constraint and state transitions
- **FR-012 to FR-019**: ChapterRecommendation (computed) + ChapterMetadata (stored)
- **FR-020 to FR-023**: PersonalizedSection uses user_profiles from Phase 4A (no new entity needed)
- **FR-024 to FR-028**: RAG chatbot uses user_profiles from Phase 4A (no new entity needed)
- **FR-029 to FR-032**: Integration with existing Better Auth and ProfileService

All entities are optimized for Neon Postgres free tier (estimated total storage: ~50KB per user for 1000 users = ~50MB, well within 0.5GB limit).
