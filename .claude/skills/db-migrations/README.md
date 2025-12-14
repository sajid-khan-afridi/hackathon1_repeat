# db-migrations

Manage PostgreSQL database schema migrations for users, profiles, chat history, and caching tables.

## Overview

The db-migrations skill provides:
- Creation of version-controlled migration files
- Running pending migrations
- Rollback support
- Migration status tracking
- Complete schema generation

## Usage

### As a CLI Tool

```bash
# Create a new migration
node index.js create_migration --name "initial_schema"

# Run all pending migrations
node index.js run_migrations

# Check migration status
node index.js status

# Rollback last migration
node index.js rollback

# Rollback to specific version
node index.js rollback --version "20240101000000_initial"

# Generate complete schema file
node index.js generate_schema --output schema.sql
```

### As a Module

```javascript
import DbMigrations from './index.js';

const migrations = new DbMigrations({
  projectRoot: process.cwd(),
  migrationsDir: './migrations'
});

// Create migration
const result = await migrations.createMigration({
  name: 'add_user_preferences',
  tables: ['users', 'user_profiles']
});

// Run migrations
const applied = await migrations.runMigrations();
console.log('Applied:', applied.migrations_applied);

// Check status
const status = await migrations.status();
console.log('Pending:', status.pending);
```

## Database Tables

### users
User authentication and account information.

| Column | Type | Description |
|--------|------|-------------|
| id | UUID | Primary key |
| email | VARCHAR(255) | Unique email address |
| password_hash | VARCHAR(255) | Hashed password |
| name | VARCHAR(255) | Display name |
| created_at | TIMESTAMP | Creation timestamp |
| updated_at | TIMESTAMP | Last update timestamp |

### user_profiles
User profile data for content personalization.

| Column | Type | Description |
|--------|------|-------------|
| id | UUID | Primary key |
| user_id | UUID | Reference to users table |
| experience_level | VARCHAR(20) | beginner, intermediate, advanced |
| ros_familiarity | VARCHAR(20) | none, basic, proficient |
| hardware_access | VARCHAR(20) | simulation_only, partial_lab, full_lab |
| learning_goal | VARCHAR(20) | career, research, hobby |
| preferred_language | VARCHAR(10) | python, cpp, both |
| content_language | VARCHAR(5) | en, ur |
| profile_hash | VARCHAR(64) | Hash for caching |

### chat_history
Chat conversation history for RAG chatbot.

| Column | Type | Description |
|--------|------|-------------|
| id | UUID | Primary key |
| user_id | UUID | Reference to users table |
| chat_id | UUID | Conversation session ID |
| role | VARCHAR(20) | user, assistant, system |
| content | TEXT | Message content |
| sources | JSONB | Retrieved source documents |
| tokens_used | INTEGER | Token count for the message |

### personalization_cache
Cached personalized content.

| Column | Type | Description |
|--------|------|-------------|
| chapter_id | VARCHAR(255) | Chapter identifier |
| profile_hash | VARCHAR(64) | User profile hash |
| content | TEXT | Cached personalized content |
| metadata | JSONB | Additional metadata |
| expires_at | TIMESTAMP | Cache expiration |

### translation_cache
Cached translations.

| Column | Type | Description |
|--------|------|-------------|
| chapter_id | VARCHAR(255) | Chapter identifier |
| source_language | VARCHAR(5) | Source language code |
| target_language | VARCHAR(5) | Target language code |
| content_hash | VARCHAR(64) | Content hash for cache key |
| translated_content | TEXT | Translated content |

## Integration

This skill integrates with:
- **neon-postgres**: Applies migrations to Neon PostgreSQL database
- **better-auth-setup**: Creates users table for authentication
- **user-profiling**: Creates user_profiles table
- **rag-pipeline**: Creates chat_history table
- **content-adapter**: Creates personalization_cache table
- **urdu-translator**: Creates translation_cache table

## Environment Variables

| Variable | Required | Description |
|----------|----------|-------------|
| DATABASE_URL | Yes | PostgreSQL connection string |
| DATABASE_POOL_MAX | No | Max connection pool size (default: 10) |

## Migration File Format

```sql
-- Migration: migration_name
-- Version: 20240101120000_migration_name
-- Created: 2024-01-01T12:00:00.000Z

-- ==================== UP ====================

CREATE TABLE example (...);

-- ==================== DOWN ====================
-- To rollback, run the following SQL:
-- DROP TABLE IF EXISTS example CASCADE;
```
