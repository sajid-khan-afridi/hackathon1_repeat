# rag-pipeline

End-to-end RAG (Retrieval-Augmented Generation) pipeline orchestrating vector search, context retrieval, and AI response generation.

## Overview

The rag-pipeline skill provides:
- Semantic search across indexed textbook content
- Context-aware AI response generation
- Chat history persistence and continuation
- Confidence scoring for responses
- Token usage tracking

## Usage

### As a CLI Tool

```bash
# Simple query
node index.js query --question "How do I create a ROS publisher?"

# Query with module filter
node index.js query --question "What is Isaac Sim?" --module 2

# Query with user tracking
node index.js query --question "Explain URDF" --user_id "user-123"

# Continue a chat session
node index.js continue --chat_id "chat-uuid" --question "Tell me more"

# Get user's chat sessions
node index.js sessions --user_id "user-123"
```

### As a Module

```javascript
import RagPipeline from './index.js';

const pipeline = new RagPipeline({
  collectionName: 'robotics_textbook',
  topK: 5,
  chatModel: 'gpt-4o-mini'
});

// Simple query
const result = await pipeline.query({
  userQuery: 'How do I create a ROS subscriber?',
  userId: 'user-123'
});

console.log('Answer:', result.answer);
console.log('Sources:', result.sources);
console.log('Confidence:', result.confidence);

// Query with filters
const filteredResult = await pipeline.query({
  userQuery: 'Explain sensor configuration',
  filters: { module: 2 },
  topK: 3
});

// Continue chat
const followUp = await pipeline.continueChat({
  chatId: result.chat_id,
  userId: 'user-123',
  userQuery: 'Can you show me an example?'
});
```

## Workflow

1. **Validate** user query and parameters
2. **Retrieve chat history** from Neon PostgreSQL (if enabled)
3. **Search Qdrant** vectorstore for relevant context
4. **Rank and filter** retrieved documents
5. **Generate AI response** using OpenAI with context
6. **Save chat exchange** to PostgreSQL
7. **Return formatted response** with sources

## API Reference

### query(options)

Process a user query through the RAG pipeline.

**Parameters:**
| Option | Type | Required | Description |
|--------|------|----------|-------------|
| userQuery | string | Yes | User's question |
| userId | string | No | User ID for chat persistence |
| topK | number | No | Number of chunks to retrieve (default: 5) |
| filters | object | No | Metadata filters ({module, chapter_id, tags}) |
| includeChatHistory | boolean | No | Include previous messages (default: true) |
| streamResponse | boolean | No | Stream response in real-time (default: false) |

**Returns:**
```json
{
  "success": true,
  "answer": "To create a ROS publisher...",
  "sources": [
    {
      "chapter_id": "docs-module-1-chapter-1",
      "title": "ROS Publishers",
      "score": 0.89,
      "excerpt": "A publisher sends messages..."
    }
  ],
  "confidence": 0.85,
  "chat_id": "uuid",
  "tokens_used": {
    "retrieval": 100,
    "generation": 500,
    "total": 600
  }
}
```

### continueChat(options)

Continue an existing chat session with context.

**Parameters:**
| Option | Type | Required | Description |
|--------|------|----------|-------------|
| chatId | string | Yes | Existing chat session ID |
| userId | string | No | User ID for verification |
| userQuery | string | Yes | Follow-up question |

### getUserChatSessions(userId, limit)

Get recent chat sessions for a user.

## Integration

This skill integrates with:
- **qdrant-vectorstore**: Performs semantic search for context retrieval
- **openai-agents-sdk**: Generates AI responses with retrieved context
- **neon-postgres**: Stores and retrieves chat history
- **content-indexer**: Uses indexed content for retrieval
- **fastapi-backend**: Exposed as /api/query endpoint

## Environment Variables

| Variable | Required | Description |
|----------|----------|-------------|
| QDRANT_URL | Yes | Qdrant server URL |
| QDRANT_API_KEY | No | Qdrant API key (required for cloud) |
| OPENAI_API_KEY | Yes | OpenAI API key |
| DATABASE_URL | No | PostgreSQL for chat history |
| EMBEDDING_MODEL | No | OpenAI model (default: text-embedding-3-small) |

## Configuration Options

| Option | Default | Description |
|--------|---------|-------------|
| collectionName | robotics_textbook | Qdrant collection name |
| topK | 5 | Number of context chunks to retrieve |
| chatModel | gpt-4o-mini | OpenAI chat model |

## Confidence Scoring

Confidence is calculated as the average of top-3 search scores:
- **0.8-1.0**: High confidence - exact match or very relevant
- **0.6-0.8**: Medium confidence - related content found
- **0.4-0.6**: Low confidence - tangentially related
- **< 0.4**: Very low - may not be in textbook

## Chat History

When `userId` is provided and `DATABASE_URL` is set:
- User and assistant messages are persisted
- Previous messages provide conversation context
- Chat sessions can be continued with `continueChat()`
- Sessions can be listed with `getUserChatSessions()`
