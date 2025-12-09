# Neon PostgreSQL Skill

A comprehensive skill for managing Neon Serverless PostgreSQL database operations, including user management, chat history, and personalization data.

## Features

- **Connection Pooling**: Optimized connection management with automatic retry logic
- **Type-Safe Operations**: Full CRUD operations with parameter validation
- **Security**: Row-level security, audit logging, rate limiting, and input sanitization
- **Migrations**: Version-controlled database schema management
- **Performance**: Connection pooling, slow query logging, and optimized queries

## Installation

```bash
npm install
```

## Environment Variables

```env
# Required
NEON_DATABASE_URL=postgresql://username:password@host/dbname?sslmode=require
NEON_DATABASE_URL_UNPOOLED=postgresql://username:password@host/dbname?sslmode=require

# Optional
DB_POOL_TIMEOUT=30000
DB_MAX_CONNECTIONS=10
```

## Quick Start

```javascript
import { NeonPostgresSkill } from './index.js';

const db = new NeonPostgresSkill({
  connectionString: process.env.NEON_DATABASE_URL,
  maxConnections: 10
});

// Initialize database
await db.initialize();

// Create a new user
const user = await db.users.create({
  email: 'user@example.com',
  password_hash: 'hashed_password'
});

// Add user profile
await db.userProfiles.create({
  user_id: user.data.id,
  experience_level: 'intermediate',
  ros_familiarity: 'basic',
  hardware_access: 'simulation',
  learning_goal: 'career',
  preferred_language: 'python'
});

// Add chat message
await db.chatHistory.add({
  user_id: user.data.id,
  session_id: 'session-uuid',
  message: 'Hello, how can I learn robotics?',
  role: 'user'
});

// Close connection
await db.close();
```

## API Reference

### Core Operations

#### Execute Custom Query
```javascript
const result = await db.execute({
  operation: 'select',
  table_name: 'users',
  query: { email: 'user@example.com' }
});
```

#### Insert Record
```javascript
await db.execute({
  operation: 'insert',
  table_name: 'chat_history',
  data: {
    user_id: 'user-uuid',
    session_id: 'session-uuid',
    message: 'Hello',
    role: 'user'
  }
});
```

#### Update Record
```javascript
await db.execute({
  operation: 'update',
  table_name: 'user_profiles',
  data: { experience_level: 'advanced' },
  where: { user_id: 'user-uuid' }
});
```

#### Delete Record
```javascript
await db.execute({
  operation: 'delete',
  table_name: 'chat_history',
  where: { session_id: 'session-uuid' }
});
```

### Helper Methods

#### User Operations
```javascript
// Create user
const user = await db.users.create({
  email: 'user@example.com',
  password_hash: 'hashed_password'
});

// Find user by email
const found = await db.users.findByEmail('user@example.com');

// Update user
await db.users.update(user.id, { email: 'new@example.com' });

// Delete user
await db.users.delete(user.id);
```

#### Chat History Operations
```javascript
// Get chat session
const messages = await db.chatHistory.getBySession('session-uuid');

// Get user's chat history
const history = await db.chatHistory.getByUser('user-uuid');

// Delete old messages
await db.chatHistory.deleteOldMessages('2024-01-01');
```

#### Personalization Cache Operations
```javascript
// Set cached content
await db.personalizationCache.set({
  chapter_id: 'chapter-1',
  user_profile_hash: 'hash-value',
  adapted_content: 'Customized content...',
  expires_at: new Date(Date.now() + 86400000) // 24 hours
});

// Get cached content
const cached = await db.personalizationCache.get('chapter-1', 'hash-value');

// Clear expired cache
await db.personalizationCache.clearExpired();
```

## Database Schema

### Users Table
```sql
CREATE TABLE users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  password_hash VARCHAR(255) NOT NULL,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);
```

### User Profiles Table
```sql
CREATE TABLE user_profiles (
  user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
  experience_level VARCHAR(20) CHECK (experience_level IN ('beginner', 'intermediate', 'advanced')),
  ros_familiarity VARCHAR(20) CHECK (ros_familiarity IN ('none', 'basic', 'proficient')),
  hardware_access VARCHAR(20) CHECK (hardware_access IN ('simulation', 'jetson', 'full_lab')),
  learning_goal VARCHAR(20) CHECK (learning_goal IN ('career', 'research', 'hobby')),
  preferred_language VARCHAR(10) CHECK (preferred_language IN ('python', 'cpp', 'both'))
);
```

### Chat History Table
```sql
CREATE TABLE chat_history (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id) ON DELETE SET NULL,
  session_id UUID NOT NULL,
  message TEXT NOT NULL,
  role VARCHAR(10) CHECK (role IN ('user', 'assistant')),
  context_text TEXT,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);
```

### Personalization Cache Table
```sql
CREATE TABLE personalization_cache (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  chapter_id VARCHAR(255) NOT NULL,
  user_profile_hash VARCHAR(64) NOT NULL,
  adapted_content TEXT NOT NULL,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
  expires_at TIMESTAMP WITH TIME ZONE NOT NULL
);
```

## Security Features

### Row-Level Security (RLS)
- Users can only access their own data
- Configurable policies for different tables
- Automatic context setting for authenticated users

### Audit Logging
```javascript
import { SecurityManager } from './src/security.js';

const security = new SecurityManager(db.pool);

// Log access
await security.logAuditEvent('READ', userId, {
  tableName: 'users',
  recordId: 'user-uuid',
  ipAddress: '192.168.1.1'
});
```

### Rate Limiting
```javascript
// Check rate limit for API calls
const rateLimit = await security.checkRateLimit('api-key-123', 100, 60000);

if (!rateLimit.allowed) {
  console.log('Rate limit exceeded');
}
```

### Input Validation
```javascript
// Validate email
if (!security.validateEmail(email)) {
  throw new Error('Invalid email format');
}

// Validate UUID
if (!security.validateUUID(userId)) {
  throw new Error('Invalid user ID');
}
```

## Migrations

Run migrations to set up the database schema:

```bash
npm run migrate
```

Or programmatically:

```javascript
const { MigrationManager } = await import('./src/migration.js');
const migration = new MigrationManager(db.pool);
await migration.migrate();
```

## Error Handling

The skill includes comprehensive error handling:

- **Connection Errors**: Automatic retry with exponential backoff
- **Query Timeouts**: Configurable timeout with fallback
- **Pool Exhaustion**: Graceful degradation and queueing
- **Validation Errors**: Detailed error messages

```javascript
try {
  const result = await db.execute(operation);
} catch (error) {
  if (error.message.includes('connection')) {
    // Handle connection errors
    console.error('Database connection failed:', error);
  } else if (error.message.includes('validation')) {
    // Handle validation errors
    console.error('Invalid data:', error);
  }
}
```

## Performance Optimization

### Connection Pooling
- Default: 10 connections (Neon free tier limit)
- Configurable pool size
- Automatic connection recycling

### Query Optimization
- Parameterized queries to prevent SQL injection
- Slow query logging (>1 second)
- Index optimization for common queries

### Caching Strategy
- Personalization cache with TTL
- Session-based chat history caching
- Automatic cleanup of expired entries

## Best Practices

1. **Always Use Transactions** for multi-table operations
2. **Validate Inputs** before database operations
3. **Use Connection Pooling** to manage connections efficiently
4. **Implement Rate Limiting** for public APIs
5. **Log Audit Events** for security-sensitive operations
6. **Clean Up** expired cache entries regularly
7. **Monitor Pool Stats** to avoid connection exhaustion

## Example: Complete User Registration Flow

```javascript
import { NeonPostgresSkill, SecurityManager } from './index.js';

const db = new NeonPostgresSkill();
const security = new SecurityManager(db.pool);

async function registerUser(email, password, profile) {
  await db.initialize();

  try {
    // Validate input
    if (!security.validateEmail(email)) {
      throw new Error('Invalid email format');
    }

    // Check if user exists
    const existing = await db.users.findByEmail(email);
    if (existing.data.length > 0) {
      throw new Error('User already exists');
    }

    // Hash password
    const passwordHash = security.hashPassword(password);

    // Create user
    const user = await db.execute({
      operation: 'insert',
      table_name: 'users',
      data: { email, password_hash: passwordHash }
    });

    // Create profile
    await db.userProfiles.create({
      user_id: user.data.id,
      ...profile
    });

    // Log registration
    await security.logAuditEvent('USER_REGISTER', user.data.id, {
      tableName: 'users',
      recordId: user.data.id
    });

    return { success: true, userId: user.data.id };
  } finally {
    await db.close();
  }
}
```