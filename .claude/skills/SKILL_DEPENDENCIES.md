# Skill Dependencies Graph

This document maps the dependencies and relationships between all Claude Code skills in this project.

## Dependency Graph

```
Infrastructure Layer:
├── env-setup (generates .env files)
├── db-migrations (manages database schema)
└── docusaurus-init (initializes Docusaurus project)
    └── github-pages-deploy (deploys to GitHub Pages)

Data Layer:
├── neon-postgres (PostgreSQL operations)
│   ├── better-auth-setup (uses users table)
│   ├── user-profiling (uses user_profiles table)
│   ├── content-adapter (uses personalization_cache table)
│   └── urdu-translator (uses translation_cache table)
│
└── qdrant-vectorstore (vector database operations)
    ├── content-indexer (stores embeddings)
    └── rag-pipeline (retrieves context)

Content Layer:
├── mdx-writer (creates MDX files)
│   └── content-indexer (indexes MDX for search)
│       └── qdrant-vectorstore
│
├── content-adapter (personalizes content)
│   ├── user-profiling (receives profile data)
│   ├── urdu-translator (translates when content_language='ur')
│   └── neon-postgres (caches personalized content)
│
└── urdu-translator (translates to Urdu)
    └── neon-postgres (caches translations)

AI & RAG Layer:
├── openai-agents-sdk (OpenAI Chat Completions)
│   ├── qdrant-vectorstore (generates embeddings)
│   └── rag-pipeline (generates responses)
│
└── rag-pipeline (end-to-end RAG orchestration)
    ├── qdrant-vectorstore (retrieves context)
    ├── openai-agents-sdk (generates answers)
    ├── neon-postgres (stores chat history)
    └── fastapi-backend (exposed as API endpoint)

Authentication Layer:
├── better-auth-setup (frontend auth)
│   ├── neon-postgres (stores users)
│   └── user-profiling (triggers profile collection)
│
└── fastapi-backend (backend API with JWT validation)
    ├── better-auth-setup (validates JWT tokens)
    ├── rag-pipeline (exposes /api/query endpoint)
    ├── neon-postgres (data operations)
    └── qdrant-vectorstore (vector search)

UI Layer:
├── react-components (React component generator)
│   ├── contextual-keyword-chips (can be integrated)
│   └── docusaurus-init (creates components in src/)
│
└── contextual-keyword-chips (keyword extraction UI)
    └── openai-agents-sdk (extracts keywords from responses)
```

## Skill Categories

### **Infrastructure (3 skills)**
- env-setup
- db-migrations
- docusaurus-init
- github-pages-deploy

### **Database (2 skills)**
- neon-postgres
- qdrant-vectorstore

### **Content (4 skills)**
- mdx-writer
- content-adapter
- content-indexer
- urdu-translator

### **AI & RAG (2 skills)**
- openai-agents-sdk
- rag-pipeline

### **Authentication (2 skills)**
- better-auth-setup
- fastapi-backend

### **User Management (1 skill)**
- user-profiling

### **UI Components (2 skills)**
- react-components
- contextual-keyword-chips

## Critical Paths

### **Content Creation → Search**
```
mdx-writer → content-indexer → qdrant-vectorstore → rag-pipeline
```

### **User Authentication → Personalization**
```
better-auth-setup → user-profiling → content-adapter → (personalized content)
```

### **RAG Chatbot Query Flow**
```
User Query → fastapi-backend → rag-pipeline → qdrant-vectorstore (search)
                                            → openai-agents-sdk (generate)
                                            → neon-postgres (store history)
                                            → Response
```

### **Setup & Deployment**
```
env-setup → db-migrations → docusaurus-init → mdx-writer → content-indexer
                                                          → github-pages-deploy
```

## Skill Integration Matrix

| Skill | Depends On | Used By |
|-------|-----------|---------|
| env-setup | - | All skills (environment vars) |
| db-migrations | neon-postgres | better-auth-setup, user-profiling, content-adapter, urdu-translator, rag-pipeline |
| docusaurus-init | - | mdx-writer, github-pages-deploy, react-components |
| github-pages-deploy | docusaurus-init | - |
| neon-postgres | - | better-auth-setup, user-profiling, content-adapter, urdu-translator, rag-pipeline |
| qdrant-vectorstore | openai-agents-sdk | content-indexer, rag-pipeline, fastapi-backend |
| mdx-writer | docusaurus-init | content-indexer, content-adapter |
| content-adapter | user-profiling, neon-postgres, urdu-translator | - |
| content-indexer | mdx-writer, qdrant-vectorstore, openai-agents-sdk | rag-pipeline |
| urdu-translator | neon-postgres, openai-agents-sdk | content-adapter |
| openai-agents-sdk | - | qdrant-vectorstore, rag-pipeline, content-indexer, urdu-translator, contextual-keyword-chips |
| rag-pipeline | qdrant-vectorstore, openai-agents-sdk, neon-postgres | fastapi-backend |
| better-auth-setup | neon-postgres | user-profiling, fastapi-backend |
| user-profiling | neon-postgres, better-auth-setup | content-adapter |
| fastapi-backend | better-auth-setup, neon-postgres, qdrant-vectorstore, rag-pipeline | - |
| react-components | docusaurus-init | contextual-keyword-chips |
| contextual-keyword-chips | openai-agents-sdk | react-components |

## Environment Variable Dependencies

### **Core Services**
- **DATABASE_URL**: neon-postgres, db-migrations, fastapi-backend
- **QDRANT_URL**: qdrant-vectorstore, content-indexer, rag-pipeline, fastapi-backend
- **OPENAI_API_KEY**: openai-agents-sdk, qdrant-vectorstore, content-indexer, rag-pipeline, urdu-translator

### **Authentication**
- **AUTH_SECRET**: better-auth-setup, fastapi-backend
- **AUTH_URL**: better-auth-setup

### **Optional**
- **QDRANT_API_KEY**: qdrant-vectorstore (cloud only)
- **DATABASE_POOL_MAX**: neon-postgres
- **EMBEDDING_MODEL**: qdrant-vectorstore, openai-agents-sdk

## Resolution Order for Setup

When setting up the project from scratch, follow this order:

1. **env-setup** - Generate .env file with all required variables
2. **db-migrations** - Create database tables
3. **docusaurus-init** - Initialize Docusaurus project
4. **better-auth-setup** - Set up authentication
5. **mdx-writer** - Create content files
6. **content-indexer** - Index content into Qdrant
7. **rag-pipeline** - Test RAG functionality
8. **fastapi-backend** - Create API endpoints
9. **react-components** - Generate UI components
10. **github-pages-deploy** - Deploy to production

## Conflict Resolution

### **No Circular Dependencies**
The dependency graph is acyclic (DAG), ensuring clean builds and no circular import issues.

### **Shared Type Definitions**
All skills use the shared `UserProfile` type from `.claude/skills/shared/types/UserProfile.ts`.

### **Database Schema Coordination**
All database operations go through `neon-postgres` to prevent schema conflicts.

### **API Integration**
Frontend auth (better-auth-setup) and backend API (fastapi-backend) share the same `AUTH_SECRET` for JWT validation.
