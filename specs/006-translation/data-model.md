# Data Model: Phase 5 Translation

**Feature**: 006-translation
**Date**: 2025-12-27
**Status**: Complete

## Entities

### 1. GlossaryTerm

Represents a technical term in the bilingual glossary.

**Storage**: Static JSON file (`src/data/glossary.json`)

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Unique identifier (kebab-case, e.g., "ros-2") |
| english | string | Yes | English term (e.g., "ROS 2") |
| urduTransliteration | string | Yes | Urdu script transliteration |
| definition | string | Yes | English definition (1-2 sentences) |
| definitionUrdu | string | Yes | Urdu definition |
| category | string | Yes | Category for filtering (e.g., "Robotics Frameworks") |
| relatedTerms | string[] | No | Array of related term IDs |

**Validation Rules**:
- `id` must be unique across all terms
- `english` must not be empty
- `category` must be one of predefined categories
- `relatedTerms` must reference valid term IDs

**Example**:
```json
{
  "id": "ros-2",
  "english": "ROS 2",
  "urduTransliteration": "آر او ایس ٹو",
  "definition": "Robot Operating System 2 - the next-generation robotics middleware",
  "definitionUrdu": "روبوٹ آپریٹنگ سسٹم 2 - اگلی نسل کا روبوٹکس مڈل ویئر",
  "category": "Robotics Frameworks",
  "relatedTerms": ["roscore", "ros-node", "ros-topic"]
}
```

---

### 2. TranslationCache

Metadata for cached translations (stored alongside translated MDX files).

**Storage**: JSON sidecar file (`docs/i18n/ur/.../chapter-1.meta.json`)

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| chapterId | string | Yes | Chapter identifier (e.g., "module-1/chapter-1") |
| sourceHash | string | Yes | MD5 hash of source English content |
| translatedAt | string | Yes | ISO 8601 timestamp of translation |
| sourceFile | string | Yes | Path to source MDX file |
| translatedFile | string | Yes | Path to translated MDX file |
| termCount | number | Yes | Number of technical terms preserved |

**Validation Rules**:
- `sourceHash` must be 32-character hex string (MD5)
- `translatedAt` must be valid ISO 8601 date
- `termCount` must be >= 0

**State Transitions**:
- **Valid**: `sourceHash` matches current source file hash
- **Stale**: `sourceHash` differs from current source file hash (requires re-translation)

**Example**:
```json
{
  "chapterId": "module-1/chapter-1",
  "sourceHash": "a1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6",
  "translatedAt": "2025-12-27T10:30:00Z",
  "sourceFile": "docs/module-1/chapter-1.mdx",
  "translatedFile": "docs/i18n/ur/docusaurus-plugin-content-docs/current/module-1/chapter-1.mdx",
  "termCount": 15
}
```

---

### 3. LanguagePreference

User's language preference (for authenticated users).

**Storage**: Neon Postgres (`user_preferences` table extension)

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| userId | string | Yes | Foreign key to users table |
| preferredLanguage | string | Yes | Language code ("en" or "ur") |
| updatedAt | string | Yes | ISO 8601 timestamp of last update |

**Validation Rules**:
- `preferredLanguage` must be "en" or "ur"
- `userId` must reference valid user

**SQL Schema Extension**:
```sql
-- Add to existing user_preferences table
ALTER TABLE user_preferences
ADD COLUMN preferred_language VARCHAR(2) DEFAULT 'en'
CHECK (preferred_language IN ('en', 'ur'));
```

---

### 4. GlossaryCategory

Predefined categories for organizing glossary terms.

**Storage**: Static TypeScript enum (compile-time validation)

| Category | Description |
|----------|-------------|
| Robotics Frameworks | ROS, ROS 2, Isaac, Gazebo |
| Programming Languages | Python, C++, JavaScript |
| Hardware Components | URDF, sensors, actuators |
| AI/ML Concepts | Neural networks, reinforcement learning |
| Mathematics | Kinematics, dynamics, control theory |
| Simulation | Gazebo, Isaac Sim, physics engines |

**TypeScript Definition**:
```typescript
export enum GlossaryCategory {
  RoboticsFrameworks = 'Robotics Frameworks',
  ProgrammingLanguages = 'Programming Languages',
  HardwareComponents = 'Hardware Components',
  AIMLConcepts = 'AI/ML Concepts',
  Mathematics = 'Mathematics',
  Simulation = 'Simulation',
}
```

---

## Relationships

```text
┌─────────────────┐
│ GlossaryTerm    │
├─────────────────┤
│ id              │──┐
│ english         │  │ relatedTerms[]
│ category        │  │ (self-reference)
│ relatedTerms[]  │◀─┘
└─────────────────┘

┌─────────────────┐     ┌─────────────────┐
│ TranslationCache│     │ MDX Source File │
├─────────────────┤     ├─────────────────┤
│ chapterId       │────▶│ (docs/*.mdx)    │
│ sourceHash      │     └─────────────────┘
│ sourceFile      │
└─────────────────┘

┌─────────────────┐     ┌─────────────────┐
│LanguagePreference│    │ User (auth)     │
├─────────────────┤     ├─────────────────┤
│ userId          │────▶│ id              │
│ preferredLanguage│    │ email           │
└─────────────────┘     └─────────────────┘
```

## Data Volume Estimates

| Entity | Expected Count | Storage Size |
|--------|----------------|--------------|
| GlossaryTerm | 50-100 terms | ~50 KB (JSON) |
| TranslationCache | 10 chapters | ~5 KB (JSON metadata) |
| LanguagePreference | Per user | ~100 bytes/user |

**Free Tier Impact**: Minimal. All translation content is static files. Only `LanguagePreference` uses Neon Postgres, well within 0.5GB limit.
