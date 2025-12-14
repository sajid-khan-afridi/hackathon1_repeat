# env-setup

Generate and validate `.env` files with all required environment variables for the robotics textbook project.

## Overview

The env-setup skill provides a standardized way to:
- Generate new `.env` files with all required variables
- Validate existing `.env` files against project requirements
- Update `.env` files with new variables
- Create `.env.example` templates for documentation

## Usage

### As a CLI Tool

```bash
# Generate a new .env file
node index.js generate --env development --output .env

# Validate an existing .env file
node index.js validate --path .env

# Generate a template file
node index.js template --output .env.example

# Update specific variables
node index.js update --path .env --DATABASE_URL "postgres://..."
```

### As a Module

```javascript
import EnvSetup from './index.js';

const envSetup = new EnvSetup({ projectRoot: process.cwd() });

// Generate .env file
const result = await envSetup.generate({
  environment: 'development',
  outputPath: '.env',
  overwrite: false,
  includeComments: true
});

// Validate .env file
const validation = await envSetup.validate({
  envPath: '.env'
});

if (!validation.valid) {
  console.log('Missing:', validation.missing_vars);
  console.log('Invalid:', validation.invalid_values);
}
```

## Required Variables

| Variable | Required | Description |
|----------|----------|-------------|
| `DATABASE_URL` | Yes | Neon PostgreSQL connection string |
| `QDRANT_URL` | Yes | Qdrant vector database URL |
| `QDRANT_API_KEY` | No | Qdrant API key (required for cloud) |
| `OPENAI_API_KEY` | Yes | OpenAI API key for embeddings |
| `AUTH_SECRET` | Yes | JWT signing secret (32+ chars) |
| `AUTH_URL` | Yes | Base URL for auth callbacks |
| `DATABASE_POOL_MAX` | No | Max connection pool size (default: 10) |
| `EMBEDDING_MODEL` | No | OpenAI embedding model (default: text-embedding-3-small) |
| `NODE_ENV` | No | Node environment (default: development) |

## Integration

This skill integrates with:
- **neon-postgres**: Requires `DATABASE_URL` and `DATABASE_POOL_MAX`
- **qdrant-vectorstore**: Requires `QDRANT_URL` and `QDRANT_API_KEY`
- **openai-agents-sdk**: Requires `OPENAI_API_KEY`
- **better-auth-setup**: Requires `AUTH_SECRET` and `AUTH_URL`

## Operations

### generate

Creates a new `.env` file with placeholder values and auto-generated secrets.

**Options:**
- `environment`: Target environment (development, production, test)
- `outputPath`: Output file path (default: `.env`)
- `overwrite`: Overwrite existing file (default: false)
- `includeComments`: Include descriptive comments (default: true)

### validate

Checks an existing `.env` file for missing or invalid values.

**Returns:**
- `valid`: Boolean indicating if validation passed
- `missing_vars`: Array of missing required variables
- `invalid_values`: Array of variables with invalid formats

### template

Generates a `.env.example` template file for documentation.

### update

Updates an existing `.env` file with new or modified variables while preserving existing values.

## Security Notes

- Never commit `.env` files to version control
- Always add `.env` to `.gitignore`
- Use `.env.example` as a template for documentation
- Rotate `AUTH_SECRET` periodically in production
