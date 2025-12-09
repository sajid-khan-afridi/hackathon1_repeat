# Workflow Patterns

This document provides common workflow patterns and usage examples for the Claude Code skills in this project.

## Table of Contents

1. [Initial Setup](#initial-setup)
2. [Content Creation](#content-creation)
3. [User Onboarding](#user-onboarding)
4. [RAG Chatbot Query](#rag-chatbot-query)
5. [Content Personalization](#content-personalization)
6. [Translation](#translation)
7. [Deployment](#deployment)
8. [Database Operations](#database-operations)

---

## Initial Setup

### Pattern: Project Initialization

**Goal**: Set up a new project from scratch with all required infrastructure.

**Steps**:

1. **Generate Environment Variables**
   ```
   Skill: env-setup
   Parameters:
     - operation: "generate"
     - environment: "development"
     - include_comments: true
   ```

2. **Initialize Docusaurus**
   ```
   Skill: docusaurus-init
   Parameters:
     - project_name: "robotics-textbook"
     - title: "Physical AI Robotics Course"
     - description: "Comprehensive robotics course with hands-on projects"
     - github_org: "your-org"
   ```

3. **Run Database Migrations**
   ```
   Skill: db-migrations
   Parameters:
     - operation: "run_migrations"
     - tables: ["all"]
   ```

4. **Set Up Authentication**
   ```
   Skill: better-auth-setup
   Parameters:
     - auth_providers: ["email"]
     - include_profile_questions: true
   ```

---

## Content Creation

### Pattern: Create and Index a New Chapter

**Goal**: Write a new MDX chapter and make it searchable in the RAG chatbot.

**Steps**:

1. **Create MDX Content**
   ```
   Skill: mdx-writer
   Parameters:
     - chapter_title: "Introduction to ROS"
     - module_number: 1
     - learning_objectives: [
         "Understand ROS architecture",
         "Set up a ROS workspace",
         "Create your first ROS node"
       ]
     - content_outline: "..."
     - code_examples:
         include: true
         languages: ["python"]
     - exercises:
         difficulty: "beginner"
         count: 3
   ```

2. **Index Content for Search**
   ```
   Skill: content-indexer
   Parameters:
     - operation: "index_chapter"
     - chapter_path: "docs/module-1/intro-to-ros.mdx"
     - collection_name: "robotics_textbook"
     - chunk_size: 500
   ```

### Pattern: Bulk Content Import

**Goal**: Index an entire directory of existing MDX files.

**Steps**:

1. **Bulk Index Directory**
   ```
   Skill: content-indexer
   Parameters:
     - operation: "index_directory"
     - directory_path: "docs/"
     - collection_name: "robotics_textbook"
     - chunk_size: 500
     - preserve_code_blocks: true
   ```

---

## User Onboarding

### Pattern: New User Registration with Profiling

**Goal**: Authenticate a new user and collect their profile for personalization.

**Steps**:

1. **User Signs Up** (handled by better-auth-setup frontend)
   - User submits email and password
   - Account created in `users` table

2. **Collect User Profile**
   ```
   Skill: user-profiling
   Parameters:
     - operation: "collect"
     - user_id: "uuid-from-auth"
     - profile_data:
         experience_level: "beginner"
         ros_familiarity: "none"
         hardware_access: "simulation_only"
         learning_goal: "career"
         preferred_language: "python"
         content_language: "en"
   ```

3. **Analyze Profile**
   ```
   Skill: user-profiling
   Parameters:
     - operation: "analyze"
     - user_id: "uuid-from-auth"

   Returns:
     - persona: "Beginner Professional"
     - content_level: "simplified"
     - recommended_modules: ["Module 1", "Module 2"]
   ```

---

## RAG Chatbot Query

### Pattern: End-to-End RAG Query

**Goal**: Process a user question through the full RAG pipeline.

**Steps**:

1. **Execute RAG Pipeline**
   ```
   Skill: rag-pipeline
   Parameters:
     - user_query: "How do I create a ROS publisher in Python?"
     - user_id: "uuid-from-auth"
     - collection_name: "robotics_textbook"
     - top_k: 5
     - include_chat_history: true
     - stream_response: false

   Returns:
     - answer: "To create a ROS publisher in Python..."
     - sources: [
         {
           chapter_id: "intro-to-ros",
           title: "Introduction to ROS",
           score: 0.92,
           excerpt: "..."
         }
       ]
     - confidence: 0.89
     - chat_id: "chat-uuid"
   ```

### Pattern: Filtered RAG Query

**Goal**: Search only specific modules or topics.

**Steps**:

1. **Query with Filters**
   ```
   Skill: rag-pipeline
   Parameters:
     - user_query: "Explain inverse kinematics"
     - user_id: "uuid-from-auth"
     - filters:
         module: 3
     - top_k: 3
   ```

---

## Content Personalization

### Pattern: Personalize Chapter for User

**Goal**: Adapt chapter content based on user's profile.

**Steps**:

1. **Retrieve User Profile**
   ```
   Skill: user-profiling
   Parameters:
     - operation: "retrieve"
     - user_id: "uuid-from-auth"
   ```

2. **Adapt Content**
   ```
   Skill: content-adapter
   Parameters:
     - chapter_content: "<MDX content here>"
     - user_profile: <profile_from_step_1>
     - chapter_id: "intro-to-ros"
     - use_cache: true

   Returns:
     - adapted_content: "<personalized MDX>"
     - metadata:
         difficulty: "simplified"
         reading_time_minutes: 15
         applied_adaptations: [
           "added_prerequisites",
           "simplified_terminology",
           "python_examples_only"
         ]
     - cache_hit: false
   ```

---

## Translation

### Pattern: Translate Chapter to Urdu

**Goal**: Translate technical content while preserving code and terms.

**Steps**:

1. **Translate Content**
   ```
   Skill: urdu-translator
   Parameters:
     - content: "<MDX content>"
     - source_language: "en"
     - preserve_terms: ["ROS", "Python", "API", "Gazebo"]
     - preserve_code_blocks: true
     - use_cache: true
     - chapter_id: "intro-to-ros"

   Returns:
     - translated_content: "<Urdu MDX with English code blocks>"
     - word_count: 1250
     - cache_hit: false
   ```

2. **Integrate with Content Adapter** (automatic when content_language='ur')
   ```
   Skill: content-adapter
   Parameters:
     - chapter_content: "<English MDX>"
     - user_profile:
         content_language: "ur"
         ...

   Internally calls urdu-translator when content_language='ur'
   ```

---

## Deployment

### Pattern: Deploy to GitHub Pages

**Goal**: Build and deploy Docusaurus site to GitHub Pages.

**Steps**:

1. **Set Up Deployment**
   ```
   Skill: github-pages-deploy
   Parameters:
     - repo_name: "robotics-textbook"
     - github_org: "your-org"
     - base_url: "/robotics-textbook/"
     - node_version: "20"

   Creates:
     - .github/workflows/deploy.yml
     - Updated docusaurus.config.ts
   ```

2. **Push to GitHub** (triggers auto-deployment via GitHub Actions)
   ```bash
   git add .
   git commit -m "Setup deployment"
   git push origin main
   ```

---

## Database Operations

### Pattern: Query User Profiles

**Goal**: Retrieve and filter user profile data.

**Steps**:

1. **Select with Filters**
   ```
   Skill: neon-postgres
   Parameters:
     - operation: "select"
     - table_name: "user_profiles"
     - select_columns: ["user_id", "experience_level", "learning_goal"]
     - where:
         experience_level: "beginner"
         hardware_access: "simulation_only"
     - limit: 50
   ```

### Pattern: Store Chat History

**Goal**: Save a chat exchange to the database.

**Steps**:

1. **Insert Chat Messages**
   ```
   Skill: neon-postgres
   Parameters:
     - operation: "insert"
     - table_name: "chat_history"
     - data:
         user_id: "uuid-from-auth"
         chat_id: "chat-uuid"
         role: "user"
         content: "How do I create a ROS publisher?"
         created_at: "2025-12-10T10:30:00Z"
   ```

2. **Insert Assistant Response**
   ```
   Skill: neon-postgres
   Parameters:
     - operation: "insert"
     - table_name: "chat_history"
     - data:
         user_id: "uuid-from-auth"
         chat_id: "chat-uuid"
         role: "assistant"
         content: "To create a ROS publisher..."
         sources: [...]
         tokens_used: 450
   ```

---

## Advanced Patterns

### Pattern: Reindex All Content

**Goal**: Rebuild the entire vector search index.

**Steps**:

1. **Delete Existing Collection**
   ```
   Skill: qdrant-vectorstore
   Parameters:
     - operation: "delete"
     - collection_name: "robotics_textbook"
     - ids: ["all"]
   ```

2. **Create New Collection**
   ```
   Skill: qdrant-vectorstore
   Parameters:
     - operation: "create_collection"
     - collection_name: "robotics_textbook"
     - vector_size: 1536
     - distance_metric: "Cosine"
   ```

3. **Reindex All Content**
   ```
   Skill: content-indexer
   Parameters:
     - operation: "reindex_all"
     - directory_path: "docs/"
     - collection_name: "robotics_textbook"
   ```

### Pattern: Profile-Based Content Recommendations

**Goal**: Recommend chapters based on user profile.

**Steps**:

1. **Analyze Profile**
   ```
   Skill: user-profiling
   Parameters:
     - operation: "analyze"
     - user_id: "uuid-from-auth"

   Returns:
     - recommended_modules: ["Module 1: Basics", "Module 2: ROS"]
   ```

2. **Search for Relevant Chapters**
   ```
   Skill: qdrant-vectorstore
   Parameters:
     - operation: "search"
     - collection_name: "robotics_textbook"
     - query: "beginner robotics fundamentals"
     - filter:
         module: 1
     - top_k: 5
   ```

---

## Error Handling Patterns

### Pattern: Graceful Degradation

**Example**: If personalization cache fails, fall back to default content.

```
Try:
  content-adapter (with use_cache: true)
Catch:
  content-adapter (with use_cache: false)
```

### Pattern: Retry with Exponential Backoff

**Example**: Retry Qdrant connection failures.

```
Attempt 1: qdrant-vectorstore → Connection Error
Wait 1s
Attempt 2: qdrant-vectorstore → Connection Error
Wait 2s
Attempt 3: qdrant-vectorstore → Success
```

---

## Performance Optimization Patterns

### Pattern: Batch Embedding Generation

**Goal**: Generate embeddings for multiple documents efficiently.

**Steps**:

1. **Batch Upsert**
   ```
   Skill: qdrant-vectorstore
   Parameters:
     - operation: "upsert"
     - collection_name: "robotics_textbook"
     - documents: [
         { id: "1", text: "...", metadata: {...} },
         { id: "2", text: "...", metadata: {...} },
         ...
       ]
   ```

### Pattern: Cache Personalized Content

**Goal**: Avoid re-personalizing content for similar user profiles.

**Steps**:

1. **Check Cache First**
   ```
   Skill: content-adapter
   Parameters:
     - chapter_content: "..."
     - user_profile: {...}
     - chapter_id: "intro-to-ros"
     - use_cache: true  ← Checks personalization_cache table
   ```

---

## Troubleshooting Workflows

### Pattern: Validate Environment Setup

**Steps**:

1. **Validate .env File**
   ```
   Skill: env-setup
   Parameters:
     - operation: "validate"
     - output_path: ".env"

   Returns:
     - validation_results:
         valid: false
         missing_vars: ["QDRANT_API_KEY"]
         invalid_values: []
   ```

2. **Fix Issues and Regenerate**
   ```
   Skill: env-setup
   Parameters:
     - operation: "generate"
     - overwrite: true
   ```

### Pattern: Debug RAG Pipeline

**Steps**:

1. **Test Each Component**
   - Test Qdrant search:
     ```
     Skill: qdrant-vectorstore
     Parameters:
       - operation: "search"
       - query: "test query"
     ```

   - Test OpenAI response:
     ```
     Skill: openai-agents-sdk
     Parameters:
       - operation: "chat"
       - user_query: "test query"
     ```

2. **Run Full Pipeline**
   ```
   Skill: rag-pipeline
   Parameters:
     - user_query: "test query"
   ```

---

## Summary

This document provides battle-tested workflow patterns for common tasks in the Physical AI Robotics Course project. Use these patterns as templates and adapt them to your specific needs.

For more details on individual skills, refer to their respective `skill.json` files.
