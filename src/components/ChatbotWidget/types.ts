/**
 * TypeScript type definitions for ChatbotWidget
 * Matches backend/app/models/query.py Pydantic models
 */

/**
 * Source citation with metadata
 * Corresponds to backend SourceCitation model
 */
export interface SourceCitation {
  chapter_id: string;
  chapter_title: string;
  relevance_score: number; // 0.0 to 1.0
  excerpt: string;
  position: number; // 1-indexed rank in results
}

/**
 * Token usage statistics for educational transparency
 * Corresponds to backend TokenUsage model
 */
export interface TokenUsage {
  input_tokens: number;
  output_tokens: number;
  total_tokens: number;
}

/**
 * Query request payload
 * Corresponds to backend QueryRequest model
 */
export interface QueryRequest {
  query: string;
  user_id?: string; // UUID string
  session_id?: string; // UUID string
  filters?: {
    module?: number; // 1-10
    difficulty?: 'beginner' | 'intermediate' | 'advanced';
    tags?: string[];
  };
  top_k?: number; // 1-10, default 5
}

/**
 * Query response from backend
 * Corresponds to backend QueryResponse model
 */
export interface QueryResponse {
  answer: string;
  sources: SourceCitation[];
  confidence: number; // 0.0 to 1.0
  session_id: string; // UUID string
  tokens_used: TokenUsage;
  filter_message?: string; // Present if adaptive filtering occurred
  suggested_terms?: string[]; // Present for off-topic queries
}

/**
 * Streaming response chunk from SSE
 * Corresponds to backend StreamChunk model
 */
export interface StreamChunk {
  chunk: string; // Text chunk (empty string if done)
  done: boolean; // Whether streaming is complete
  sources?: SourceCitation[]; // Only present when done=true
  confidence?: number; // Only present when done=true
  session_id?: string; // Only present when done=true
  tokens_used?: TokenUsage; // Only present when done=true
}

/**
 * Message in the chat UI
 * Extended from backend ChatMessage with UI-specific fields
 */
export interface Message {
  id: string; // Generated client-side or from backend
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: SourceCitation[]; // Only for assistant messages
  confidence?: number; // Only for assistant messages
  tokens_used?: TokenUsage; // Only for assistant messages
  filter_message?: string; // Only for assistant messages
  suggested_terms?: string[]; // Only for assistant messages
}

/**
 * Chat state managed by useReducer
 */
export interface ChatState {
  messages: Message[];
  sessionId: string | null;
  isLoading: boolean;
  isStreaming: boolean; // Whether a response is currently streaming
  streamingContent: string; // Accumulated streaming content
  streamingMessageId: string | null; // ID of message being streamed
  error: string | null;
  streamError: string | null; // Separate error for stream interruptions
  moduleFilter: number | null; // Selected module (1-10) or null for all modules
  rateLimitState: RateLimitState;
}

/**
 * Actions for chat state reducer
 */
export type ChatAction =
  | { type: 'ADD_USER_MESSAGE'; payload: { content: string } }
  | { type: 'ADD_ASSISTANT_MESSAGE'; payload: QueryResponse }
  | { type: 'START_STREAMING'; payload: { messageId: string } }
  | { type: 'APPEND_STREAM_CHUNK'; payload: { chunk: string } }
  | { type: 'COMPLETE_STREAM'; payload: { sources: SourceCitation[]; confidence: number; session_id: string; tokens_used: TokenUsage } }
  | { type: 'INTERRUPT_STREAM'; payload: { error?: string } }
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_ERROR'; payload: string | null }
  | { type: 'SET_SESSION_ID'; payload: string }
  | { type: 'SET_MODULE_FILTER'; payload: number | null }
  | { type: 'SET_RATE_LIMIT'; payload: RateLimitState }
  | { type: 'CLEAR_HISTORY' }
  | { type: 'RETRY_STREAM' };

/**
 * Error response from backend
 */
export interface ErrorResponse {
  detail: string;
  error_code?: string;
}

/**
 * Rate limit error response
 */
export interface RateLimitError {
  error: string;
  message: string;
  retry_after: number; // seconds
}

/**
 * Rate limit state
 */
export interface RateLimitState {
  isRateLimited: boolean;
  retryAfter: number; // seconds remaining
  resetTime: Date | null; // when the rate limit resets
}
