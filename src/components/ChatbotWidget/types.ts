/**
 * Type definitions for the RAG Chatbot Widget
 */

export interface FilterParams {
  module?: number; // 1-10
  difficulty?: 'beginner' | 'intermediate' | 'advanced';
  tags?: string[];
}

export interface SourceCitation {
  content: string;
  chapter: number;
  section?: string;
  module?: number;
  relevanceScore: number;
  pageUrl?: string;
}

export interface TokenUsage {
  promptTokens: number;
  completionTokens: number;
  totalTokens: number;
}

export interface QueryRequest {
  query: string;
  sessionId?: string;
  filters?: FilterParams;
}

export interface QueryResponse {
  answer: string;
  confidence: number;
  sources: SourceCitation[];
  sessionId: string;
  suggestedQuestions?: string[];
  filterMessage?: string;
  tokenUsage: TokenUsage;
  responseTime: number;
}

export interface ErrorResponse {
  error: string;
  code: string;
  details?: Record<string, any>;
  suggestedAction?: string;
}

export interface OffTopicResponse {
  message: string;
  suggestedTopics: string[];
  suggestedQueries: string[];
}

export interface LowConfidenceResponse {
  answer: string;
  confidence: number;
  warning: string;
  suggestedRephrase: string;
  sources: SourceCitation[];
  sessionId: string;
  tokenUsage: TokenUsage;
  responseTime: number;
}

export interface RateLimitResponse {
  error: string;
  limit: number;
  resetAfter: number;
  retryAfter: string;
}

export type ApiResponse =
  | QueryResponse
  | OffTopicResponse
  | LowConfidenceResponse
  | ErrorResponse
  | RateLimitResponse;

export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: SourceCitation[];
  confidence?: number;
  tokenUsage?: TokenUsage;
  filterMessage?: string;
  warning?: string;
  suggestedRephrase?: string;
  suggestedTopics?: string[];
  suggestedQueries?: string[];
  error?: string;
}

export interface RateLimitState {
  isLimited: boolean;
  limit: number;
  resetAfter: number;
  resetTime: Date | null;
}

export interface ChatState {
  messages: ChatMessage[];
  isLoading: boolean;
  isStreaming: boolean;
  sessionId: string | null;
  error: string | null;
  suggestedQuestions: string[];
  rateLimit: RateLimitState | null;
}

export interface ChatbotWidgetProps {
  /**
   * Base URL for the API
   * @default "http://localhost:8000"
   */
  apiUrl?: string;

  /**
   * Widget title
   * @default "Ask a Question"
   */
  title?: string;

  /**
   * Placeholder text for the input field
   * @default "Ask about robotics..."
   */
  placeholder?: string;

  /**
   * Maximum number of messages to display
   * @default 50
   */
  maxMessages?: number;

  /**
   * Whether to show suggested questions
   * @default true
   */
  showSuggestions?: boolean;

  /**
   * Whether to show token usage
   * @default true
   */
  showTokenUsage?: boolean;

  /**
   * Whether to allow module filtering
   * @default true
   */
  allowFiltering?: boolean;

  /**
   * Custom CSS class for the widget
   */
  className?: string;

  /**
   * Callback when a message is sent
   */
  onMessageSent?: (message: string) => void;

  /**
   * Callback when a response is received
   */
  onResponseReceived?: (response: ApiResponse) => void;

  /**
   * Callback when an error occurs
   */
  onError?: (error: string) => void;
}

export type StreamingEvent =
  | { type: 'start'; sessionId: string }
  | { type: 'chunk'; chunk: string }
  | {
      type: 'end';
      answer: string;
      confidence: number;
      sources: SourceCitation[];
      tokenUsage: TokenUsage;
      responseTime: number;
      filterMessage?: string;
    }
  | { type: 'off_topic'; message: string; suggestedTopics: string[]; suggestedQueries: string[] }
  | { type: 'error'; error: string };

export interface ConfidenceIndicatorProps {
  confidence: number;
  showLabel?: boolean;
  size?: 'small' | 'medium' | 'large';
}

export interface ModuleFilterProps {
  onFilterChange: (filters: FilterParams) => void;
  filters: FilterParams;
}

export interface MessageListProps {
  messages: ChatMessage[];
  isStreaming?: boolean;
  maxMessages?: number;
  className?: string;
}

export interface ChatInputProps {
  onSendMessage: (message: string) => void;
  onClearHistory: () => void;
  disabled?: boolean;
  placeholder?: string;
  className?: string;
}

export interface SourceCitationsProps {
  sources: SourceCitation[];
  className?: string;
}

export interface LoadingStateProps {
  message?: string;
  className?: string;
}
