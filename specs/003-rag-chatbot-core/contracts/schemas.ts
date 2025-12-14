/**
 * RAG Chatbot API TypeScript Types
 *
 * Auto-generated from OpenAPI spec: openapi.yaml
 * Feature: 003-rag-chatbot-core
 * Version: 1.0.0
 */

// ============================================================================
// Request Types
// ============================================================================

/**
 * Query filters for narrowing search scope
 */
export interface QueryFilters {
  /** Filter by textbook module number (1-10) */
  module?: number;
  /** Filter by content difficulty level */
  difficulty?: 'beginner' | 'intermediate' | 'advanced';
  /** Filter by topic tags */
  tags?: string[];
}

/**
 * Request payload for /query endpoint
 */
export interface QueryRequest {
  /** User's question text (1-500 characters) */
  query: string;
  /** Existing session ID to continue conversation */
  sessionId?: string;
  /** Search filters */
  filters?: QueryFilters;
  /** Number of source documents to retrieve (1-10, default 5) */
  topK?: number;
}

// ============================================================================
// Response Types
// ============================================================================

/**
 * Token usage breakdown for a response
 */
export interface TokenUsage {
  /** Input tokens consumed */
  input: number;
  /** Output tokens generated */
  output: number;
  /** Total tokens used */
  total: number;
}

/**
 * Source citation from textbook content
 */
export interface SourceCitation {
  /** Chapter identifier (e.g., 'module-1/chapter-2') */
  chapterId: string;
  /** Human-readable chapter title */
  chapterTitle: string;
  /** Vector similarity score (0.0-1.0) */
  relevanceScore: number;
  /** Relevant text excerpt from chapter (max 1000 chars) */
  excerpt: string;
}

/**
 * Response from /query endpoint
 */
export interface QueryResponse {
  /** AI-generated answer text */
  answer: string;
  /** Source citations from textbook */
  sources: SourceCitation[];
  /**
   * Answer confidence score:
   * - 0.0-0.2: No relevant content (answer declined)
   * - 0.2-0.3: Low confidence (warning shown)
   * - 0.3-0.7: Medium confidence
   * - 0.7-1.0: High confidence
   */
  confidence: number;
  /** Chat session identifier */
  sessionId: string;
  /** Token usage breakdown */
  tokensUsed: TokenUsage;
  /** Optional message about filter behavior */
  filterMessage?: string;
}

/**
 * Server-Sent Event chunk for streaming responses
 */
export interface StreamChunk {
  /** Text chunk of the answer */
  chunk?: string;
  /** Whether streaming is complete */
  done?: boolean;
  /** Sources (only sent when done=true) */
  sources?: SourceCitation[];
  /** Confidence score (only sent when done=true) */
  confidence?: number;
  /** Token usage (only sent when done=true) */
  tokensUsed?: TokenUsage;
  /** Error message if stream interrupted */
  error?: string;
}

// ============================================================================
// Chat Session Types
// ============================================================================

/**
 * Message role in chat conversation
 */
export type MessageRole = 'user' | 'assistant';

/**
 * Individual chat message
 */
export interface ChatMessage {
  /** Unique message identifier */
  id: string;
  /** Message author role */
  role: MessageRole;
  /** Message text content */
  content: string;
  /** Token usage (assistant messages only) */
  tokensUsed?: TokenUsage;
  /** Confidence score (assistant messages only) */
  confidence?: number;
  /** Source citations (assistant messages only) */
  sources?: SourceCitation[];
  /** Message creation timestamp */
  createdAt: string;
}

/**
 * Chat session with message history
 */
export interface ChatSessionResponse {
  /** Session identifier */
  sessionId: string;
  /** Message history (newest first) */
  messages: ChatMessage[];
  /** Session creation timestamp */
  createdAt: string;
  /** Last activity timestamp */
  lastActivityAt: string;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * Error codes for programmatic handling
 */
export type ErrorCode =
  | 'INVALID_QUERY'
  | 'QUERY_TOO_LONG'
  | 'RATE_LIMIT_EXCEEDED'
  | 'SESSION_NOT_FOUND'
  | 'UNAUTHORIZED'
  | 'LLM_TIMEOUT'
  | 'VECTOR_STORE_UNAVAILABLE'
  | 'DATABASE_ERROR'
  | 'INTERNAL_ERROR';

/**
 * Fallback FAQ item when vector store unavailable
 */
export interface FallbackFaqItem {
  question: string;
  answer: string;
}

/**
 * Error details
 */
export interface ErrorDetail {
  /** Error code for programmatic handling */
  code: ErrorCode;
  /** Human-readable error message */
  message: string;
  /** Request correlation ID for debugging */
  correlationId: string;
  /** Seconds until retry is allowed (for rate limits) */
  retryAfter?: number;
  /** Fallback FAQ content (when vector store unavailable) */
  fallbackFaq?: FallbackFaqItem[];
}

/**
 * Error response wrapper
 */
export interface ErrorResponse {
  error: ErrorDetail;
}

// ============================================================================
// Health Check Types
// ============================================================================

/**
 * Service health status
 */
export type HealthStatus = 'healthy' | 'unhealthy' | 'degraded';

/**
 * Dependency health status
 */
export type DependencyHealth = 'healthy' | 'unhealthy';

/**
 * Dependencies health status
 */
export interface DependenciesHealth {
  database: DependencyHealth;
  vectorStore: DependencyHealth;
  llmService: DependencyHealth;
}

/**
 * Health check response
 */
export interface HealthResponse {
  /** Overall service status */
  status: HealthStatus;
  /** API version */
  version: string;
  /** Dependencies health status */
  dependencies: DependenciesHealth;
  /** Timestamp of health check */
  timestamp: string;
}

// ============================================================================
// Rate Limit Types
// ============================================================================

/**
 * Rate limit response headers
 */
export interface RateLimitHeaders {
  /** Seconds until rate limit resets */
  'Retry-After': number;
  /** Total allowed requests per window */
  'X-RateLimit-Limit': number;
  /** Remaining requests in current window */
  'X-RateLimit-Remaining': number;
  /** Unix timestamp when window resets */
  'X-RateLimit-Reset': number;
}

// ============================================================================
// API Client Types
// ============================================================================

/**
 * Options for streaming query
 */
export interface StreamQueryOptions {
  /** Callback for each text chunk */
  onChunk?: (chunk: string) => void;
  /** Callback when streaming completes */
  onComplete?: (response: Omit<QueryResponse, 'answer'> & { fullAnswer: string }) => void;
  /** Callback on error */
  onError?: (error: ErrorDetail) => void;
  /** AbortController signal for cancellation */
  signal?: AbortSignal;
}

/**
 * API configuration
 */
export interface ApiConfig {
  /** Base URL for API requests */
  baseUrl: string;
  /** JWT token for authenticated requests */
  authToken?: string;
  /** Session token for anonymous users */
  sessionToken?: string;
  /** Request timeout in milliseconds */
  timeout?: number;
}

// ============================================================================
// Confidence Thresholds
// ============================================================================

/**
 * Confidence score thresholds for UI behavior
 */
export const CONFIDENCE_THRESHOLDS = {
  /** Below this: decline to answer */
  DECLINE: 0.2,
  /** Below this: show warning banner */
  LOW_WARNING: 0.3,
  /** Above this: high confidence */
  HIGH: 0.7,
} as const;

/**
 * Determine confidence level from score
 */
export function getConfidenceLevel(
  confidence: number
): 'declined' | 'low' | 'medium' | 'high' {
  if (confidence < CONFIDENCE_THRESHOLDS.DECLINE) return 'declined';
  if (confidence < CONFIDENCE_THRESHOLDS.LOW_WARNING) return 'low';
  if (confidence < CONFIDENCE_THRESHOLDS.HIGH) return 'medium';
  return 'high';
}

// ============================================================================
// Rate Limit Constants
// ============================================================================

/**
 * Rate limit configuration
 */
export const RATE_LIMITS = {
  /** Queries per hour for anonymous users */
  ANONYMOUS: 10,
  /** Queries per hour for authenticated users */
  AUTHENTICATED: 50,
  /** Rate limit window in seconds */
  WINDOW_SECONDS: 3600,
} as const;

// ============================================================================
// Validation Helpers
// ============================================================================

/**
 * Maximum query length
 */
export const MAX_QUERY_LENGTH = 500;

/**
 * Maximum excerpt length
 */
export const MAX_EXCERPT_LENGTH = 1000;

/**
 * Validate query request
 */
export function validateQueryRequest(request: QueryRequest): string[] {
  const errors: string[] = [];

  if (!request.query || request.query.trim().length === 0) {
    errors.push('Query cannot be empty');
  }

  if (request.query && request.query.length > MAX_QUERY_LENGTH) {
    errors.push(`Query exceeds ${MAX_QUERY_LENGTH} character limit`);
  }

  if (request.topK !== undefined && (request.topK < 1 || request.topK > 10)) {
    errors.push('topK must be between 1 and 10');
  }

  if (request.filters?.module !== undefined) {
    if (request.filters.module < 1 || request.filters.module > 10) {
      errors.push('Module filter must be between 1 and 10');
    }
  }

  return errors;
}
