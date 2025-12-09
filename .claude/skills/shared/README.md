# Shared Types and Utilities

This directory contains shared type definitions and utilities used across multiple skills.

## UserProfile.ts

The canonical definition of the user profile structure used throughout the application.

### Usage

```typescript
import { UserProfile, validateUserProfile, generateProfileHash } from '@skills/shared/types/UserProfile';

// Create a user profile
const profile: UserProfile = {
  experience_level: 'intermediate',
  ros_familiarity: 'basic',
  hardware_access: 'partial_lab',
  learning_goal: 'career',
  preferred_language: 'python',
  content_language: 'en'
};

// Validate the profile
const validation = validateUserProfile(profile);
if (!validation.valid) {
  console.error('Validation errors:', validation.errors);
}

// Generate hash for caching
const hash = generateProfileHash(profile);
```

## Integration with Skills

All skills that work with user profiles should import from this shared definition:

- **better-auth-setup**: Uses UserProfile for profile questions
- **user-profiling**: Uses UserProfile for data collection and storage
- **content-adapter**: Uses UserProfile for content personalization
- **neon-postgres**: Uses UserProfile schema for database tables
- **fastapi-backend**: Uses UserProfile for API request/response models

## Extending the Profile

If you need to add new fields:

1. Update `UserProfile` interface in this file
2. Update database schema in `neon-postgres/migrations/`
3. Update all dependent skills
4. Update validation functions
5. Document the change in the integration guide
