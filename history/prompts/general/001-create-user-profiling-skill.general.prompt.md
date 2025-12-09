---
ID: 001
TITLE: Create User Profiling Skill
STAGE: general
DATE_ISO: 2025-12-09
SURFACE: agent
MODEL: claude-sonnet-4-5-20250929
FEATURE: none
BRANCH: main
USER: user
COMMAND:
LABELS: ["user-profiling", "skill", "personalization", "data-collection"]
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
FILES_YAML:
  - .claude/skills/user-profiling/skill.ts
  - .claude/skills/user-profiling/schema.ts
  - .claude/skills/user-profiling/database.ts
  - .claude/skills/user-profiling/analysis.ts
  - .claude/skills/user-profiling/README.md
  - .claude/skills/user-profiling/examples.ts
  - .claude/skills/user-profiling/package.json
  - .claude/skills/user-profiling/skill.md
TESTS_YAML:
OUTCOME: Completed
EVALUATION: Successfully created a comprehensive user profiling skill with all required functionality including profile collection, validation, analysis, database storage, and documentation.
---

PROMPT_TEXT:
Skill\_name: `user-profiling`

Create a skill for collecting and managing user profile information during signup.

### Purpose:

Gather user background information during registration to enable content personalization throughout their learning journey.

### Inputs:

- operation: "collect" | "retrieve" | "update" | "analyze"
- user_id?: string (for retrieve/update/analyze)
- responses?: ProfileResponse[] (for collect)

### Profile Questions Schema:

```
Questions:
├── Q1: Programming Experience
│   ├── field: "experience_level"
│   ├── type: "single_select"
│   ├── question: "What is your programming experience level?"
│   └── options:
│       ├── beginner: "Beginner (< 1 year coding)"
│       ├── intermediate: "Intermediate (1-3 years)"
│       └── advanced: "Advanced (3+ years)"
│
├── Q2: ROS Familiarity
│   ├── field: "ros_familiarity"
│   ├── type: "single_select"
│   ├── question: "How familiar are you with ROS (Robot Operating System)?"
│   └── options:
│       ├── none: "No experience"
│       ├── basic: "Basic (completed tutorials)"
│       └── proficient: "Proficient (built ROS projects)"
│
├── Q3: Hardware Access
│   ├── field: "hardware_access"
│   ├── type: "single_select"
│   ├── question: "What robotics hardware do you have access to?"
│   └── options:
│       ├── simulation: "Simulation only (no physical hardware)"
│       ├── jetson: "Jetson kit with sensors"
│       └── full_lab: "Full robot lab (robot + sensors + workstation)"
│
├── Q4: Learning Goal
│   ├── field: "learning_goal"
│   ├── type: "single_select"
│   ├── question: "What is your primary goal for this course?"
│   └── options:
│       ├── career: "Career advancement in robotics"
│       ├── research: "Academic research"
│       └── hobby: "Personal interest / hobby"
│
└── Q5: Preferred Code Language
    ├── field: "preferred_language"
    ├── type: "single_select"
    ├── question: "Which programming language do you prefer for code examples?"
    └── options:
        ├── python: "Python"
        ├── cpp: "C++"
        └── both: "Both (show all examples)"
```

### Outputs:

- Validated user profile object
- Profile completeness score (0-100%)
- Recommended learning path based on profile
- Profile hash (for caching personalized content)

### Profile Analysis:

```
analyze(user_id) returns:
├── persona: string (e.g., "Beginner Hobbyist", "Advanced Researcher")
├── content_level: "simplified" | "standard" | "advanced"
├── recommended_modules: string[] (priority order)
├── estimated_completion_time: string
└── personalization_hints: {
    show_prerequisites: boolean,
    include_research_papers: boolean,
    hardware_specific_content: string,
    code_language_filter: string
}
```

### Validation Rules:

- All 5 questions must be answered (required)
- Validate option values against allowed enums
- Sanitize any free-text inputs (if added later)

### Storage:

- Store in `user_profiles` table (Neon Postgres)
- Generate profile hash for cache lookup: `SHA256(experience_level + ros_familiarity + hardware_access)`
- Update `updated_at` timestamp on any change

### Privacy Considerations:

- Profile data is used only for personalization
- User can view and delete their profile
- No profile data shared with third parties
- Anonymized analytics only

RESPONSE_TEXT:
Successfully created the `user-profiling` skill with comprehensive functionality including:

1. **Schema & Validation** (`schema.ts`): Defined all types, questions, and validation logic with support for profile completeness scoring and hash generation.

2. **Database Layer** (`database.ts`): Implemented full PostgreSQL integration with connection pooling, CRUD operations, and proper indexing for performance.

3. **Profile Analysis** (`analysis.ts`): Created sophisticated analysis engine that generates personas, content levels, module recommendations, and personalization hints based on user profiles.

4. **Main Skill Handler** (`skill.ts`): Built the core skill interface supporting all operations (collect, retrieve, update, analyze) with comprehensive error handling and validation.

5. **Documentation**:
   - `README.md`: Complete usage guide and API documentation
   - `skill.md`: Formal skill specification for integration
   - `examples.ts`: Practical usage examples covering all scenarios

6. **Package Configuration** (`package.json`): Proper dependency management for PostgreSQL and TypeScript support.

The skill successfully handles all requirements:
- Collects 5-structured questions about programming experience, ROS familiarity, hardware access, learning goals, and language preferences
- Generates personalized learning paths with module recommendations
- Provides content complexity adjustment based on user experience
- Supports hardware-specific content filtering
- Implements privacy-compliant data storage with user control
- Includes comprehensive validation and error handling