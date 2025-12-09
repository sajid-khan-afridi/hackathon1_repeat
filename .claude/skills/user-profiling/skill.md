# Skill: User Profiling

**Skill Name:** `user-profiling`

**Purpose:** Collect and manage user profile information during signup to enable content personalization throughout their learning journey.

## Description

This skill is responsible for gathering comprehensive user background information through a structured 5-question survey. The collected data is used to:
- Create personalized learning paths
- Recommend appropriate modules and content
- Adjust content complexity based on experience level
- Provide hardware-specific learning resources
- Filter code examples by language preference

## Input Schema

```typescript
{
  operation: "collect" | "retrieve" | "update" | "analyze",
  user_id?: string,           // Required for retrieve/update/analyze
  responses?: ProfileResponse[] // Required for collect/update
}
```

### Profile Questions

1. **Programming Experience** (`experience_level`)
   - `beginner`: Beginner (< 1 year coding)
   - `intermediate`: Intermediate (1-3 years)
   - `advanced`: Advanced (3+ years)

2. **ROS Familiarity** (`ros_familiarity`)
   - `none`: No experience
   - `basic`: Basic (completed tutorials)
   - `proficient`: Proficient (built ROS projects)

3. **Hardware Access** (`hardware_access`)
   - `simulation`: Simulation only (no physical hardware)
   - `jetson`: Jetson kit with sensors
   - `full_lab`: Full robot lab (robot + sensors + workstation)

4. **Learning Goal** (`learning_goal`)
   - `career`: Career advancement in robotics
   - `research`: Academic research
   - `hobby`: Personal interest / hobby

5. **Preferred Code Language** (`preferred_language`)
   - `python`: Python
   - `cpp`: C++
   - `both`: Both (show all examples)

## Output Schema

### Successful Response

```typescript
{
  success: true,
  data: {
    profile: UserProfile,        // Complete profile object
    analysis: ProfileAnalysis,   // Personalization recommendations
    completeness?: number,       // 0-100% (for collect)
    user_id?: string,           // Generated ID (for collect)
    updated?: boolean           // True for update operations
  }
}
```

### Profile Analysis

```typescript
{
  persona: string,              // "Beginner Hobbyist", "Advanced Researcher", etc.
  content_level: "simplified" | "standard" | "advanced",
  recommended_modules: string[], // Priority-ordered module list
  estimated_completion_time: string, // e.g., "16 weeks"
  personalization_hints: {
    show_prerequisites: boolean,
    include_research_papers: boolean,
    hardware_specific_content: string,
    code_language_filter: string
  }
}
```

### Error Response

```typescript
{
  success: false,
  errors: string[],            // Validation or system errors
  warnings?: string[]          // Incomplete profile warnings
}
```

## Operations

### 1. Collect (`operation: "collect"`)
- **Purpose**: Create a new user profile
- **Requirements**: All 5 questions must be answered
- **Output**: Created profile with analysis

### 2. Retrieve (`operation: "retrieve"`)
- **Purpose**: Get existing user profile
- **Requirements**: `user_id` must be provided
- **Output**: Complete profile data

### 3. Update (`operation: "update"`)
- **Purpose**: Modify existing profile
- **Requirements**: `user_id` and at least one response
- **Output**: Updated profile with new analysis

### 4. Analyze (`operation: "analyze"`)
- **Purpose**: Generate recommendations for existing profile
- **Requirements**: `user_id` must be provided
- **Output**: Profile analysis without full profile data

## Validation Rules

- All 5 questions are required for profile creation
- Response values must match predefined options
- Profile hash generated from key fields for caching
- Completeness score calculated for partial profiles

## Storage

- **Database**: Neon PostgreSQL
- **Table**: `user_profiles`
- **Indexes**: profile_hash, experience_level, ros_familiarity, hardware_access
- **Fields**: user_id, 5 profile fields, profile_hash, timestamps

## Privacy & Security

- Profile data used exclusively for personalization
- Users can view and delete their profiles
- No data shared with third parties
- Anonymized analytics only
- Profile hashes enable content caching without exposing PII

## Use Cases

1. **New User Registration**: Collect initial profile during signup
2. **Progress Checkpoint**: Update profile as skills advance
3. **Content Recommendation**: Analyze profile for module suggestions
4. **A/B Testing**: Group users by similar profiles
5. **Learning Path Optimization**: Adjust paths based on hardware access

## Integration Points

- **Learning Management System**: Provides profile data for content filtering
- **Content Delivery**: Uses analysis for module recommendations
- **Analytics Platform**: Consumes anonymized profile insights
- **User Dashboard**: Displays profile and learning progress