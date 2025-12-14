# ADR-007: RAG LLM Architecture Stack

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-14
- **Feature:** 003-rag-chatbot-core
- **Context:** Need to select and integrate LLM and embedding technologies for the RAG chatbot that enables students to ask questions about robotics textbook content. Requirements include: accurate responses grounded in source material (no hallucinations), streaming for perceived performance, cost efficiency for educational use case, and maintainability within free-tier constraints.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: YES - Long-term consequence for response quality, latency, cost, and API surface
     2) Alternatives: YES - Multiple viable LLM providers, orchestration frameworks, and streaming patterns
     3) Scope: YES - Cross-cutting concern affecting backend services, frontend state, and user experience
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Adopt OpenAI Chat Completions API with GPT-4o-mini for response generation, text-embedding-3-small for query embedding, and Server-Sent Events for streaming responses.

- **LLM Model**: GPT-4o-mini (cost-effective for educational Q&A)
- **Embedding Model**: text-embedding-3-small (1536 dimensions, $0.00002/1K tokens)
- **API Pattern**: OpenAI Chat Completions API (direct integration, no orchestration framework)
- **Streaming**: Server-Sent Events (SSE) for progressive response rendering
- **Grounding Strategy**: Strict system prompt enforcing source-only answers + weighted confidence scoring
- **Fallback**: Static FAQ JSON for graceful degradation when services unavailable

## Consequences

### Positive

- **Cost Efficiency**: GPT-4o-mini at ~$0.15/1M input tokens is 10-50x cheaper than GPT-4o/Claude
- **Quality**: Excellent for educational Q&A when properly prompted with retrieved context
- **Streaming UX**: First token appears in <1s, keeping users engaged during 2-3s generation
- **Simplicity**: Direct OpenAI API integration without LangChain overhead
- **Single Vendor**: Both embedding and generation from OpenAI simplifies API key management
- **Native Support**: FastAPI StreamingResponse + OpenAI SDK both support SSE natively
- **Grounding**: System prompt + confidence thresholds prevent hallucination
- **Extensibility**: Tool calling available in Chat Completions API for future enhancements

### Negative

- **Vendor Lock-in**: Tied to OpenAI for both embedding and generation
- **API Dependency**: External service dependency for core functionality
- **Rate Limits**: Subject to OpenAI tier limits (mitigated by user-level rate limiting)
- **Cost Unpredictability**: Usage-based pricing can spike with high traffic
- **No Fine-tuning**: Using base GPT-4o-mini without domain-specific training
- **Model Deprecation Risk**: OpenAI may deprecate models requiring migration

## Alternatives Considered

**Alternative Stack A: LangChain + Multiple Providers**
- Framework: LangChain for RAG orchestration
- LLM: Choice of OpenAI, Anthropic, or local models
- Benefits: Provider flexibility, pre-built RAG patterns, easy switching
- Rejected: Unnecessary abstraction for single-provider use case, added complexity, slower iteration

**Alternative Stack B: OpenAI Agents SDK with Tool Calling**
- Pattern: Agents SDK for structured tool use
- Benefits: Built-in tool calling, error handling, extensibility
- Rejected: Over-engineering for Q&A use case without multi-tool workflows, higher latency

**Alternative Stack C: Self-hosted LLM (Llama 3.1)**
- Model: Llama 3.1 70B on RunPod/Modal
- Benefits: No API costs, full control, no rate limits
- Rejected: Infrastructure complexity, deployment cost exceeds API costs at expected scale, maintenance burden

**Alternative Stack D: Anthropic Claude**
- Model: Claude 3.5 Sonnet via API
- Benefits: Excellent reasoning, longer context, strong grounding
- Rejected: Higher cost ($3/1M input), second vendor to manage, no embedding model (still need OpenAI)

**Alternative Streaming: WebSocket**
- Pattern: Full-duplex WebSocket connection
- Benefits: Bidirectional communication, real-time updates
- Rejected: Overkill for unidirectional streaming, more complex infrastructure

## References

- Feature Spec: specs/003-rag-chatbot-core/spec.md
- Implementation Plan: specs/003-rag-chatbot-core/plan.md
- Research: specs/003-rag-chatbot-core/research.md (Sections 1-3)
- Related ADRs: ADR-001 (Frontend Platform Stack)
