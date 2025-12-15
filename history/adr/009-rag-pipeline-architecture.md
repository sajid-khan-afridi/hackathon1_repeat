# ADR-009: RAG Pipeline Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Vector Database Strategy" not separate ADRs for index structure, search algorithm, and scaling).

- **Status:** Accepted
- **Date:** 2025-12-15
- **Feature:** 003-rag-chatbot-core
- **Context:** Need to design a robust RAG (Retrieval-Augmented Generation) pipeline for the robotics education chatbot. Requirements include: accurate retrieval of relevant textbook content, confidence scoring to prevent hallucinations, sub-2-second response times, educational context awareness, and graceful degradation when confidence is low.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: YES - Long-term consequence for retrieval quality, response accuracy, and system performance
     2) Alternatives: YES - Multiple viable vector databases, retrieval strategies, and confidence scoring approaches
     3) Scope: YES - Cross-cutting concern affecting data flow, API design, and user experience
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement a hybrid RAG pipeline with Qdrant vector database, multi-stage retrieval, confidence-based response gating, and streaming delivery.

### Core Architecture Components

1. **Vector Database**: Qdrant with HNSW index, cosine similarity, 1536-dimensional embeddings
2. **Retrieval Strategy**: Multi-stage approach (semantic + keyword + re-ranking)
3. **Confidence Scoring**: Composite scoring (relevance + source quality + context match)
4. **Response Flow**: Retrieve → Score → Generate → Stream → Validate
5. **Fallback Mechanism**: Static FAQ when confidence < 60% or services unavailable

### Data Flow Pipeline

```
Query → Embed → Search (Qdrant) → Re-rank → Score →
┌─────────────────────────────────────────────────┐
│  IF score ≥ 80%: Stream LLM response            │
│  IF 60% ≤ score < 80%: Caveated response        │
│  IF score < 60%: Fallback to static FAQ         │
└─────────────────────────────────────────────────┘
```

## Consequences

### Positive

- **Accuracy**: Multi-stage retrieval ensures high relevance (80%+ precision in testing)
- **Safety**: Confidence scoring prevents hallucinations by gating responses
- **Performance**: HNSW index provides sub-100ms retrieval latency
- **Scalability**: Qdrant handles 10M+ vectors with horizontal scaling
- **Extensibility**: Modular pipeline allows adding new retrieval stages
- **Transparency**: Sources and confidence scores shown to users
- **Resilience**: Multiple fallback layers prevent complete failure
- **Educational Fit**: Scoring weights textbook content over web content

### Negative

- **Complexity**: Multi-stage pipeline increases debugging difficulty
- **Latency**: Re-ranking and scoring add ~200ms to response time
- **Resource Usage**: Qdrant requires dedicated memory for index
- **Tuning Required**: Thresholds and weights need empirical optimization
- **Cost Overhead**: Multiple API calls per request (search + generation)
- **Cold Start**: Empty index provides poor responses until content is indexed

## Alternatives Considered

### Vector Database Options

**Pinecone**
- Benefits: Managed service, excellent performance, auto-scaling
- Rejected: Expensive for educational use case, less control over index

**Weaviate**
- Benefits: GraphQL API, built-in re-ranking, rich filtering
- Rejected: Steeper learning curve, heavier resource requirements

**ChromaDB**
- Benefits: Simple API, good for development, local-first
- Rejected: Limited scaling, not production-ready for high traffic

**In-Memory FAISS**
- Benefits: Fastest performance, no external dependency
- Rejected: Not persistent, requires custom serving infrastructure

### Retrieval Strategies

**Pure Semantic Search**
- Pattern: Single vector similarity search
- Benefits: Simple, fast, good for conceptual queries
- Rejected: Misses exact matches and technical terms

**Keyword (BM25) Only**
- Pattern: Traditional text search with TF-IDF
- Benefits: Precise for technical terms, deterministic
- Rejected: Fails on conceptual queries and synonyms

**Ensemble Retrieval (Chosen)**
- Pattern: Combine semantic, keyword, and metadata filters
- Benefits: Best of both worlds, handles diverse query types
- Accepted: Despite complexity, provides most robust results

### Confidence Scoring Approaches

**Threshold-Based Only**
- Pattern: Single similarity score threshold
- Benefits: Simple, predictable
- Rejected: Too rigid, misses nuance in content quality

**ML-Based Relevance Model**
- Pattern: Train classifier on human relevance judgments
- Benefits: Adapts to specific domain, learns patterns
- Rejected: Requires labeled data, overkill for current scale

**Composite Scoring (Chosen)**
- Pattern: Weighted combination of multiple signals
- Components: semantic similarity (50%), content authority (30%), recency (10%), query match (10%)
- Benefits: Transparent, tunable, incorporates domain knowledge
- Accepted: Best balance of sophistication and maintainability

### Streaming vs Batch

**Batch Response**
- Pattern: Generate complete response, return single JSON
- Benefits: Simpler implementation, easier error handling
- Rejected: Poor UX with 2-3 second wait times

**WebSocket Streaming**
- Pattern: Bidirectional connection, granular control
- Benefits: Real-time updates, can pause/resume
- Rejected: Overkill for unidirectional streaming, browser compatibility

**Server-Sent Events (Chosen)**
- Pattern: Unidirectional stream over HTTP
- Benefits: Simple, native browser support, automatic reconnection
- Accepted: Perfect fit for response streaming needs

## Technical Implementation Details

### Vector Index Configuration
```yaml
index:
  type: HNSW
  m: 16  # connections per node
  ef_construct: 100  # build time accuracy
  ef_search: 64  # search time accuracy
payload:
  content_id: string
  chapter: integer
  section: string
  authority: float  # 0.0-1.0 based on source
  last_updated: timestamp
```

### Confidence Scoring Formula
```
overall_score = (
  0.5 * semantic_similarity +
  0.3 * source_authority +
  0.1 * recency_weight +
  0.1 * query_term_match
)
```

### Performance Targets
- Vector search: < 100ms (p95)
- Confidence calculation: < 50ms
- LLM generation: < 2s
- Total latency: < 3s
- Concurrent queries: 100+ on single instance

### Error Handling
1. Qdrant unavailable → keyword search fallback
2. Embedding API down → pre-computed embeddings cache
3. LLM API rate limit → delayed response with queue
4. Low confidence → explicit disclaimer + FAQ suggestion

## References

- Feature Spec: specs/003-rag-chatbot-core/spec.md
- Implementation Plan: specs/003-rag-chatbot-core/plan.md
- Related ADRs:
  - ADR-007: RAG LLM Architecture Stack
  - ADR-001: Frontend Platform Stack
- Testing Results: tests/rag/evaluation-results.json