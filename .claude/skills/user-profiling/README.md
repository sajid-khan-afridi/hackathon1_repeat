# User Profiling Skill

A comprehensive skill for collecting and managing user profile information during signup to enable content personalization throughout the learning journey.

## Features

- **Profile Collection**: Gather user background information through structured questions
- **Profile Validation**: Ensure all required data is collected and validated
- **Profile Analysis**: Generate personalized learning recommendations based on user profile
- **Database Storage**: Persist user profiles in Neon PostgreSQL
- **Privacy Compliant**: User data used only for personalization with deletion capabilities

## Installation

1. Ensure you have Node.js and PostgreSQL installed
2. Set up your database connection string in environment variables:
   ```bash
   export DATABASE_URL="postgresql://username:password@host:port/database"
   ```

## Usage

### Basic Operations

```typescript
import { userProfilingSkill } from './skill';

// Collect a new user profile
const collectResult = await userProfilingSkill({
  operation: 'collect',
  user_id: 'user_123',
  responses: [
    { field: 'experience_level', value: 'intermediate' },
    { field: 'ros_familiarity', value: 'basic' },
    { field: 'hardware_access', value: 'jetson' },
    { field: 'learning_goal', value: 'career' },
    { field: 'preferred_language', value: 'python' }
  ]
});

// Retrieve an existing profile
const retrieveResult = await userProfilingSkill({
  operation: 'retrieve',
  user_id: 'user_123'
});

// Update profile information
const updateResult = await userProfilingSkill({
  operation: 'update',
  user_id: 'user_123',
  responses: [
    { field: 'experience_level', value: 'advanced' }
  ]
});

// Analyze profile for recommendations
const analyzeResult = await userProfilingSkill({
  operation: 'analyze',
  user_id: 'user_123'
});
```

### Profile Questions

The skill collects information across 5 key areas:

1. **Programming Experience**
   - Beginner (< 1 year coding)
   - Intermediate (1-3 years)
   - Advanced (3+ years)

2. **ROS Familiarity**
   - No experience
   - Basic (completed tutorials)
   - Proficient (built ROS projects)

3. **Hardware Access**
   - Simulation only
   - Jetson kit with sensors
   - Full robot lab

4. **Learning Goal**
   - Career advancement in robotics
   - Academic research
   - Personal interest / hobby

5. **Preferred Code Language**
   - Python
   - C++
   - Both (show all examples)

## Profile Analysis Output

The analysis provides:

```typescript
{
  persona: "Intermediate Professional",
  content_level: "standard",
  recommended_modules: [
    "ros-fundamentals",
    "python-basics",
    "ros-concepts",
    "gazebo-simulation",
    "jetbot-setup",
    "industry-best-practices"
  ],
  estimated_completion_time: "16 weeks",
  personalization_hints: {
    show_prerequisites: false,
    include_research_papers: false,
    hardware_specific_content: "jetson-focused",
    code_language_filter: "python"
  }
}
```

## Database Schema

```sql
CREATE TABLE user_profiles (
  user_id VARCHAR(255) PRIMARY KEY,
  experience_level VARCHAR(20) NOT NULL,
  ros_familiarity VARCHAR(20) NOT NULL,
  hardware_access VARCHAR(20) NOT NULL,
  learning_goal VARCHAR(20) NOT NULL,
  preferred_language VARCHAR(20) NOT NULL,
  profile_hash VARCHAR(64) NOT NULL,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

## Privacy Considerations

- Profile data is used exclusively for content personalization
- Users can view and delete their profiles at any time
- No profile data is shared with third parties
- Only anonymized analytics are collected
- Profile hashes are used for efficient content caching

## Error Handling

The skill returns structured error messages:

```typescript
{
  success: false,
  errors: [
    "Missing required field: experience_level",
    "Invalid value for ros_familiarity: unknown"
  ]
}
```

## Performance Considerations

- Profile hash generation enables efficient content caching
- Database indexes on common query fields
- Minimal data transfer with selective field updates
- Connection pooling for database operations

## Testing

Run tests with:

```bash
npm test
```

## Dependencies

- `pg`: PostgreSQL client
- TypeScript for type safety
- Node.js environment