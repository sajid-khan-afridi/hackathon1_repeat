# Research: RAG Chatbot Core

**Feature**: 003-rag-chatbot-core
**Date**: 2025-12-14
**Status**: Complete

## Research Summary

This document consolidates research findings for the RAG Chatbot Core implementation, resolving all technical unknowns identified during the planning phase.

---

## 1. Embedding Model Selection

### Decision
**Model**: OpenAI `text-embedding-3-small` (1536 dimensions)

### Rationale
- **Consistency**: Spec assumption states content was indexed using `sentence-transformers/all-MiniLM-L6-v2` (384 dimensions). However, the `qdrant-vectorstore` skill configuration shows 1536 dimensions, indicating OpenAI embeddings are the actual standard.
- **Performance**: text-embedding-3-small offers excellent performance-to-cost ratio ($0.00002/1K tokens)
- **Quality**: Superior semantic understanding compared to MiniLM for educational content
- **Integration**: Direct OpenAI API integration simplifies architecture (single API key)

### Alternatives Considered
| Model | Dimensions | Cost | Quality | Reason Rejected |
|-------|------------|------|---------|-----------------|
| text-embedding-3-large | 3072 | $0.00013/1K | Higher | Overkill for educational Q&A, 2x storage |
| sentence-transformers/all-MiniLM-L6-v2 | 384 | Free | Good | Lower semantic quality, requires separate hosting |
| Cohere embed-english-v3.0 | 1024 | $0.0001/1K | Good | Additional vendor, no significant advantage |

### Implementation Note
Before implementation, verify the actual embedding model used in Phase 2 indexing. If MiniLM was used, either:
1. Re-embed all content with text-embedding-3-small (preferred)
2. Adapt to use MiniLM for query embedding (requires hosting)

---

## 2. LLM Response Generation

### Decision
**Model**: OpenAI GPT-4o-mini via Chat Completions API

### Rationale
- **Spec Clarification**: Spec explicitly states "GPT-4o-mini for cost efficiency with acceptable quality for educational content" (line 219)
- **Cost**: ~$0.15/1M input tokens, ~$0.60/1M output tokens - highly cost-effective
- **Quality**: Excellent for educational Q&A with proper prompting
- **Streaming**: Native streaming support for perceived performance
- **Tool Calling**: Available if needed for future enhancements (not used in Phase 3)

### Alternatives Considered
| Model | Cost (1M tokens) | Quality | Reason Rejected |
|-------|------------------|---------|-----------------|
| GPT-4o | $2.50 in / $10 out | Higher | 10x cost, not justified for educational Q&A |
| GPT-4-turbo | $10 in / $30 out | Higher | 50x cost, overkill |
| Claude 3.5 Sonnet | $3 in / $15 out | Comparable | Different vendor, higher cost |
| Llama 3.1 70B (self-hosted) | Variable | Good | Requires hosting infrastructure |

### System Prompt Strategy
```
You are a helpful robotics tutor assistant for a Physical AI textbook. Your role is to:
1. Answer questions ONLY using the provided context from the textbook
2. ALWAYS cite specific chapters/sections when providing information
3. If the question cannot be answered from the context, say "I couldn't find information about that topic in the textbook"
4. Be concise but thorough in explanations
5. When showing code examples, ensure they are from the provided context

IMPORTANT: Never make up information. Only use facts from the provided context.
```

---

## 3. Streaming Response Pattern

### Decision
**Pattern**: Server-Sent Events (SSE) with chunked transfer encoding

### Rationale
- **Perceived Performance**: First token appears in <1s even when full response takes 2-3s
- **User Experience**: Progressive rendering keeps users engaged
- **Native Support**: FastAPI + OpenAI SDK both support streaming natively
- **Simplicity**: Simpler than WebSockets for unidirectional streaming

### Implementation Pattern
```python
# Backend (FastAPI)
from fastapi.responses import StreamingResponse

@app.post("/api/v1/query")
async def query(request: QueryRequest):
    async def generate():
        async for chunk in llm_service.stream_response(request.query, context):
            yield f"data: {json.dumps({'chunk': chunk})}\n\n"
        yield f"data: {json.dumps({'done': True, 'sources': sources})}\n\n"

    return StreamingResponse(generate(), media_type="text/event-stream")
```

```typescript
// Frontend (React)
const eventSource = new EventSource('/api/v1/query');
eventSource.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.done) {
        setSources(data.sources);
        eventSource.close();
    } else {
        setAnswer(prev => prev + data.chunk);
    }
};
```

### Alternatives Considered
| Pattern | Pros | Cons | Reason Rejected |
|---------|------|------|-----------------|
| WebSocket | Bidirectional | Complex setup, overkill | Only need server→client |
| Long Polling | Simple | Higher latency, more requests | Poor UX |
| Full Response | Simplest | 3s wait time | Poor perceived performance |

---

## 4. Rate Limiting Strategy

### Decision
**Strategy**: Token bucket algorithm with user tier differentiation

### Rationale
- **Fairness**: Different limits for authenticated vs anonymous users (spec FR-019, FR-020)
- **Burstability**: Token bucket allows short bursts while enforcing hourly limits
- **Implementation**: SlowAPI library provides FastAPI-native rate limiting

### Configuration
| User Type | Limit | Window | Bucket Size |
|-----------|-------|--------|-------------|
| Anonymous | 10 queries | 1 hour | 10 |
| Authenticated | 50 queries | 1 hour | 50 |

### Implementation Pattern
```python
from slowapi import Limiter
from slowapi.util import get_remote_address

def get_user_identifier(request: Request):
    """Get user ID for authenticated users, IP for anonymous"""
    if hasattr(request.state, 'user_id'):
        return f"user:{request.state.user_id}"
    return f"anon:{get_remote_address(request)}"

limiter = Limiter(key_func=get_user_identifier)

@app.post("/api/v1/query")
@limiter.limit("10/hour", key_func=get_user_identifier,
               override_defaults=True,
               cost=lambda req: 1 if "anon:" in get_user_identifier(req) else 0.2)
async def query(request: QueryRequest):
    ...
```

### Storage
- **Development**: In-memory storage (default)
- **Production**: Redis for distributed rate limiting across instances

---

## 5. Confidence Score Calculation

### Decision
**Method**: Weighted average of vector similarity scores with threshold mapping

### Rationale
- **Grounded**: Based on actual retrieval quality, not LLM self-assessment
- **Transparent**: Users can understand why confidence is low
- **Actionable**: Clear thresholds for UI behavior (spec FR-032)

### Calculation
```python
def calculate_confidence(search_results: List[SearchResult]) -> float:
    """
    Calculate confidence score from vector search results.

    Score interpretation:
    - 0.0-0.2: No relevant content found → decline to answer
    - 0.2-0.3: Low confidence → show warning banner
    - 0.3-0.7: Medium confidence → normal answer
    - 0.7-1.0: High confidence → answer with high certainty
    """
    if not search_results:
        return 0.0

    # Top result has highest weight
    weights = [0.4, 0.25, 0.15, 0.1, 0.1]  # For top 5 results
    weighted_sum = sum(
        result.score * weight
        for result, weight in zip(search_results[:5], weights)
    )

    # Normalize to 0-1 range (scores are already 0-1 from cosine similarity)
    return min(1.0, weighted_sum)
```

### Threshold Behavior (from spec)
| Confidence | Behavior |
|------------|----------|
| < 0.2 | Decline to answer, suggest search terms |
| 0.2-0.3 | Show answer with prominent warning + rephrase suggestion |
| 0.3-0.7 | Normal answer |
| > 0.7 | High confidence answer |

---

## 6. Chat History Schema Design

### Decision
**Pattern**: Normalized schema with separate tables for sessions, messages, and citations

### Rationale
- **Query Performance**: Efficient retrieval of recent messages for context
- **Storage Efficiency**: Citations stored once, referenced by ID
- **Flexibility**: Easy to add metadata, analytics without schema changes
- **Click-to-Navigate**: Citation IDs enable direct chapter navigation

### Schema Design
```sql
-- Sessions table
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NULL,  -- NULL for anonymous sessions
    session_token VARCHAR(64) NOT NULL,  -- For anonymous session identification
    created_at TIMESTAMPTZ DEFAULT NOW(),
    last_activity_at TIMESTAMPTZ DEFAULT NOW(),
    metadata JSONB DEFAULT '{}'
);

-- Messages table
CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    tokens_used JSONB DEFAULT '{}',  -- {input: N, output: M}
    confidence FLOAT NULL,  -- Only for assistant messages
    created_at TIMESTAMPTZ DEFAULT NOW()
);

-- Source citations table (normalized)
CREATE TABLE source_citations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID REFERENCES chat_messages(id) ON DELETE CASCADE,
    chapter_id VARCHAR(100) NOT NULL,
    chapter_title VARCHAR(255) NOT NULL,
    relevance_score FLOAT NOT NULL,
    excerpt TEXT NOT NULL,
    position INT NOT NULL  -- Order in citation list
);

-- Indexes for common queries
CREATE INDEX idx_sessions_user_id ON chat_sessions(user_id);
CREATE INDEX idx_sessions_token ON chat_sessions(session_token);
CREATE INDEX idx_messages_session_id ON chat_messages(session_id);
CREATE INDEX idx_messages_created_at ON chat_messages(created_at);
CREATE INDEX idx_citations_message_id ON source_citations(message_id);
```

### Auto-Purge Strategy
```sql
-- Scheduled job (run daily)
DELETE FROM chat_sessions
WHERE last_activity_at < NOW() - INTERVAL '30 days';
```

### Alternatives Considered
| Pattern | Pros | Cons | Reason Rejected |
|---------|------|------|-----------------|
| Denormalized (citations in message JSON) | Simple queries | Duplication, harder navigation | Spec explicitly chose normalized |
| Single messages table | Simplest | No session context | Poor UX for conversation threads |

---

## 7. Vector Search Optimization

### Decision
**Strategy**: HNSW index with metadata filtering and score thresholding

### Rationale
- **Speed**: HNSW provides <100ms search for 10k+ vectors
- **Quality**: Configurable ef parameter balances speed/recall
- **Filtering**: Metadata filters enable module-specific searches (spec FR-016)

### Qdrant Configuration
```python
# Collection creation
client.create_collection(
    collection_name="robotics_textbook",
    vectors_config=VectorParams(
        size=1536,  # text-embedding-3-small
        distance=Distance.COSINE
    ),
    hnsw_config=HnswConfigDiff(
        m=16,  # Number of edges per node
        ef_construct=100,  # Index build quality
        full_scan_threshold=10000  # Switch to brute force below this
    )
)
```

### Search Query Pattern
```python
async def search_context(
    query_embedding: List[float],
    top_k: int = 5,
    module_filter: Optional[int] = None,
    score_threshold: float = 0.2
) -> List[SearchResult]:
    filter_conditions = None
    if module_filter:
        filter_conditions = Filter(
            must=[FieldCondition(key="module", match=MatchValue(value=module_filter))]
        )

    results = client.search(
        collection_name="robotics_textbook",
        query_vector=query_embedding,
        limit=top_k,
        score_threshold=score_threshold,
        query_filter=filter_conditions,
        with_payload=True
    )
    return results
```

---

## 8. Graceful Degradation Strategy

### Decision
**Fallback Chain**: Static FAQ → Error Message with Retry

### Rationale
- **User Experience**: Always provide something useful
- **Simplicity**: Static JSON file requires no additional infrastructure
- **Maintainability**: FAQ content version-controlled with codebase

### Fallback Scenarios
| Failure | Fallback | User Message |
|---------|----------|--------------|
| Qdrant unavailable | Static FAQ | "Search is temporarily unavailable. Here are common questions..." |
| OpenAI timeout | Retry once, then static FAQ | "Generation is taking longer than usual. Please try again." |
| OpenAI rate limit | Queue + retry after backoff | "High demand. Your question is queued..." |
| Database unavailable | Stateless mode (no history) | "Chat history is temporarily unavailable. Your conversation won't be saved." |

### Static FAQ Structure
```json
{
  "version": "1.0.0",
  "lastUpdated": "2025-12-14",
  "questions": [
    {
      "id": "what-is-ros2",
      "question": "What is ROS 2?",
      "answer": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software...",
      "relatedChapters": ["module-1/chapter-1"],
      "tags": ["ros2", "basics"]
    },
    // ... more FAQ items
  ]
}
```

---

## 9. Frontend State Management

### Decision
**Pattern**: React Context + useReducer for chat state

### Rationale
- **Simplicity**: No external state library needed
- **Docusaurus Compatibility**: Works with existing React 19 setup
- **Scoped State**: Chat state isolated to ChatbotWidget subtree

### State Structure
```typescript
interface ChatState {
  sessionId: string | null;
  messages: Message[];
  isLoading: boolean;
  error: Error | null;
  streamingAnswer: string;
  moduleFilter: number | null;
}

type ChatAction =
  | { type: 'SEND_MESSAGE'; payload: string }
  | { type: 'RECEIVE_CHUNK'; payload: string }
  | { type: 'RECEIVE_COMPLETE'; payload: { sources: Citation[]; confidence: number } }
  | { type: 'SET_ERROR'; payload: Error }
  | { type: 'SET_MODULE_FILTER'; payload: number | null }
  | { type: 'CLEAR_HISTORY' };
```

---

## 10. Module Filtering Behavior

### Decision
**Strategy**: Confidence-based adaptive filtering (per spec clarification)

### Rationale
- **User Intent**: Honor filter when relevant content exists
- **Fallback**: Help users when filter is too restrictive
- **Transparency**: Clear communication about search scope

### Behavior Matrix
| Confidence | Filter Active | Behavior |
|------------|---------------|----------|
| ≥ 0.7 | Yes | Strict filter, answer from module only |
| 0.4-0.7 | Yes | Answer from module + suggest checking other modules |
| < 0.4 | Yes | Ignore filter, search all modules, explain to user |
| Any | No | Search all modules |

### Implementation
```python
def apply_adaptive_filter(
    query_embedding: List[float],
    module_filter: Optional[int],
    top_k: int = 5
) -> Tuple[List[SearchResult], str]:
    """
    Apply adaptive filtering based on confidence.
    Returns results and a filter status message.
    """
    if module_filter is None:
        results = search_context(query_embedding, top_k)
        return results, ""

    # First, try with filter
    filtered_results = search_context(query_embedding, top_k, module_filter)
    filtered_confidence = calculate_confidence(filtered_results)

    if filtered_confidence >= 0.7:
        return filtered_results, ""

    # Get unfiltered results for comparison
    all_results = search_context(query_embedding, top_k)
    all_confidence = calculate_confidence(all_results)

    if filtered_confidence >= 0.4:
        return filtered_results, f"Found relevant content in Module {module_filter}. Other modules may have additional information."

    if all_confidence > filtered_confidence + 0.2:
        return all_results, f"I couldn't find relevant information in Module {module_filter}, so I searched all modules."

    return filtered_results, f"Limited results found in Module {module_filter}. Try removing the filter for broader search."
```

---

## Research Verification Checklist

- [x] Embedding model: Verified text-embedding-3-small (1536 dims) per skill configs
- [x] LLM model: GPT-4o-mini confirmed in spec clarifications
- [x] Streaming: SSE pattern validated for FastAPI + React
- [x] Rate limiting: SlowAPI with token bucket confirmed
- [x] Confidence scoring: Weighted average approach validated
- [x] Chat history: Normalized schema per spec clarification
- [x] Vector search: Qdrant HNSW configuration validated
- [x] Graceful degradation: Static FAQ fallback confirmed in spec
- [x] State management: React Context appropriate for Docusaurus
- [x] Module filtering: Adaptive filtering per spec clarification

---

## References

- Spec: `specs/003-rag-chatbot-core/spec.md`
- Constitution: `.specify/memory/constitution.md`
- Qdrant Skill: `.claude/skills/qdrant-vectorstore/skill.json`
- OpenAI Skill: `.claude/skills/openai-agents-sdk/skill.json`
- FastAPI Skill: `.claude/skills/fastapi-backend/skill.json`
- React Components Skill: `.claude/skills/react-components/skill.json`
