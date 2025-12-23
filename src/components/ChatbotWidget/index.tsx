/**
 * ChatbotWidget Component
 *
 * Main chatbot interface for RAG question answering.
 * Features:
 * - useReducer for state management (messages, loading, error, sessionId)
 * - Fetch POST to /api/v1/query with QueryRequest payload
 * - Session persistence in localStorage
 * - Clear History functionality (DELETE /api/v1/chat/sessions/{sessionId})
 * - Confidence warnings and suggested terms
 * - Error handling with retry
 * - WCAG 2.1 AA compliance (keyboard nav, ARIA, focus management)
 * - SSR compatibility check
 */

import React, { useReducer, useEffect, useCallback } from 'react';
import Link from '@docusaurus/Link';
import styles from './ChatbotWidget.module.css';
import type {
  ChatState,
  ChatAction,
  Message,
  QueryRequest,
  QueryResponse,
  ErrorResponse,
  RateLimitError,
  RateLimitState,
  StreamChunk
} from './types';
import ChatInput from './ChatInput';
import MessageList from './MessageList';
import ModuleFilter from './ModuleFilter';
import LoadingState from './LoadingState';
import { useAuthContext } from '@site/src/context/AuthContext';

// Production API URL
const PRODUCTION_API_URL = 'https://hackathon1repeat-production.up.railway.app';

// Get API URL with proper fallback chain
const getApiUrl = (): string => {
  if (typeof window !== 'undefined') {
    // Check if running on production domain (GitHub Pages) - highest priority
    if (window.location.hostname === 'sajid-khan-afridi.github.io') {
      return PRODUCTION_API_URL;
    }

    // Check global config set by chatbotConfig.ts
    if ((window as any).CHATBOT_API_URL) {
      return (window as any).CHATBOT_API_URL;
    }

    // Fallback to Docusaurus config
    const docusaurusConfig = (window as any).__DOCUSAURUS__;
    if (docusaurusConfig?.siteConfig?.customFields?.apiUrl) {
      return docusaurusConfig.siteConfig.customFields.apiUrl;
    }
  }
  // Default to localhost for development
  return 'http://localhost:8000';
};

const API_BASE_URL = getApiUrl();

const SESSION_STORAGE_KEY = 'chatbot-session-id';

/**
 * Chat state reducer
 * Manages messages, session, loading, streaming, and error states
 */
function chatReducer(state: ChatState, action: ChatAction): ChatState {
  switch (action.type) {
    case 'ADD_USER_MESSAGE':
      return {
        ...state,
        messages: [
          ...state.messages,
          {
            id: `user-${Date.now()}`,
            role: 'user',
            content: action.payload.content,
            timestamp: new Date()
          }
        ],
        error: null,
        streamError: null
      };

    case 'ADD_ASSISTANT_MESSAGE':
      return {
        ...state,
        messages: [
          ...state.messages,
          {
            id: `assistant-${Date.now()}`,
            role: 'assistant',
            content: action.payload.answer,
            timestamp: new Date(),
            sources: action.payload.sources,
            confidence: action.payload.confidence,
            tokens_used: action.payload.tokens_used,
            filter_message: action.payload.filter_message,
            suggested_terms: action.payload.suggested_terms
          }
        ],
        sessionId: action.payload.session_id,
        isLoading: false,
        isStreaming: false,
        error: null,
        streamError: null
      };

    case 'START_STREAMING':
      return {
        ...state,
        isStreaming: true,
        isLoading: false,
        streamingContent: '',
        streamingMessageId: action.payload.messageId,
        error: null,
        streamError: null,
        messages: [
          ...state.messages,
          {
            id: action.payload.messageId,
            role: 'assistant',
            content: '',
            timestamp: new Date()
          }
        ]
      };

    case 'APPEND_STREAM_CHUNK':
      if (!state.streamingMessageId) return state;

      return {
        ...state,
        streamingContent: state.streamingContent + action.payload.chunk,
        messages: state.messages.map(msg =>
          msg.id === state.streamingMessageId
            ? { ...msg, content: state.streamingContent + action.payload.chunk }
            : msg
        )
      };

    case 'COMPLETE_STREAM':
      if (!state.streamingMessageId) return state;

      return {
        ...state,
        isStreaming: false,
        streamingMessageId: null,
        streamingContent: '',
        sessionId: action.payload.session_id,
        messages: state.messages.map(msg =>
          msg.id === state.streamingMessageId
            ? {
                ...msg,
                sources: action.payload.sources,
                confidence: action.payload.confidence,
                tokens_used: action.payload.tokens_used
              }
            : msg
        )
      };

    case 'INTERRUPT_STREAM':
      return {
        ...state,
        isStreaming: false,
        streamError: action.payload.error || 'Stream was interrupted. You can retry to get the complete response.',
        isLoading: false
      };

    case 'RETRY_STREAM':
      return {
        ...state,
        streamError: null,
        isLoading: false,
        isStreaming: false
      };

    case 'SET_LOADING':
      return {
        ...state,
        isLoading: action.payload
      };

    case 'SET_ERROR':
      return {
        ...state,
        error: action.payload,
        isLoading: false,
        isStreaming: false
      };

    case 'SET_SESSION_ID':
      return {
        ...state,
        sessionId: action.payload
      };

    case 'SET_MODULE_FILTER':
      return {
        ...state,
        moduleFilter: action.payload
      };

    case 'SET_RATE_LIMIT':
      return {
        ...state,
        rateLimitState: action.payload,
        isLoading: false,
        isStreaming: false
      };

    case 'CLEAR_HISTORY':
      return {
        messages: [],
        sessionId: null,
        isLoading: false,
        isStreaming: false,
        streamingContent: '',
        streamingMessageId: null,
        error: null,
        streamError: null,
        moduleFilter: state.moduleFilter, // Preserve filter when clearing history
        rateLimitState: {
          isRateLimited: false,
          retryAfter: 0,
          resetTime: null
        }
      };

    default:
      return state;
  }
}

/**
 * Initial state factory
 */
function getInitialState(): ChatState {
  const defaultRateLimitState: RateLimitState = {
    isRateLimited: false,
    retryAfter: 0,
    resetTime: null
  };

  // SSR compatibility check
  if (typeof window === 'undefined') {
    return {
      messages: [],
      sessionId: null,
      isLoading: false,
      isStreaming: false,
      streamingContent: '',
      streamingMessageId: null,
      error: null,
      streamError: null,
      moduleFilter: null,
      rateLimitState: defaultRateLimitState
    };
  }

  // Restore session from localStorage
  const storedSessionId = localStorage.getItem(SESSION_STORAGE_KEY);

  return {
    messages: [],
    sessionId: storedSessionId,
    isLoading: false,
    isStreaming: false,
    streamingContent: '',
    streamingMessageId: null,
    error: null,
    streamError: null,
    moduleFilter: null,
    rateLimitState: defaultRateLimitState
  };
}

interface ChatbotWidgetProps {
  /** Optional initial module filter */
  moduleFilter?: number;
  /** Optional difficulty filter */
  difficultyFilter?: 'beginner' | 'intermediate' | 'advanced';
}

export default function ChatbotWidget({
  moduleFilter,
  difficultyFilter
}: ChatbotWidgetProps = {}): JSX.Element {
  const [state, dispatch] = useReducer(chatReducer, null, getInitialState);
  const { isAuthenticated } = useAuthContext();

  /**
   * Persist session ID to localStorage
   */
  useEffect(() => {
    if (typeof window !== 'undefined' && state.sessionId) {
      localStorage.setItem(SESSION_STORAGE_KEY, state.sessionId);
    }
  }, [state.sessionId]);

  /**
   * Listen for suggested term clicks from MessageList
   */
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const handleSuggestedTerm = (event: Event) => {
      const customEvent = event as CustomEvent<{ term: string }>;
      if (customEvent.detail?.term) {
        handleSubmit(customEvent.detail.term);
      }
    };

    window.addEventListener('chatbot-suggested-term', handleSuggestedTerm);
    return () => {
      window.removeEventListener('chatbot-suggested-term', handleSuggestedTerm);
    };
  }, [state.sessionId]); // Re-bind when session changes

  /**
   * Submit query with streaming support using EventSource (SSE)
   */
  const handleSubmit = useCallback(async (query: string) => {
    // Add user message to UI
    dispatch({ type: 'ADD_USER_MESSAGE', payload: { content: query } });
    dispatch({ type: 'SET_LOADING', payload: true });

    try {
      const requestBody: QueryRequest = {
        query,
        session_id: state.sessionId || undefined,
        top_k: 5
      };

      // Apply filters if provided (use state moduleFilter instead of prop)
      if (state.moduleFilter || difficultyFilter) {
        requestBody.filters = {
          module: state.moduleFilter || undefined,
          difficulty: difficultyFilter
        };
      }

      // Try streaming first
      const response = await fetch(`${API_BASE_URL}/api/v1/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'text/event-stream'  // Request streaming
        },
        body: JSON.stringify(requestBody)
      });

      if (!response.ok) {
        // Handle rate limit errors (429) specially
        if (response.status === 429) {
          try {
            const rateLimitData: RateLimitError = await response.json();
            const resetTime = new Date(Date.now() + rateLimitData.retry_after * 1000);

            dispatch({
              type: 'SET_RATE_LIMIT',
              payload: {
                isRateLimited: true,
                retryAfter: rateLimitData.retry_after,
                resetTime
              }
            });

            return; // Don't throw error, just set rate limit state
          } catch (parseError) {
            // Fallback if JSON parsing fails
            dispatch({
              type: 'SET_RATE_LIMIT',
              payload: {
                isRateLimited: true,
                retryAfter: 3600, // Default 1 hour
                resetTime: new Date(Date.now() + 3600000)
              }
            });
            return;
          }
        }

        // Handle other HTTP errors
        let errorMessage = `Server error: ${response.status}`;

        try {
          const errorData: ErrorResponse = await response.json();
          errorMessage = errorData.detail || errorMessage;
        } catch {
          // Ignore JSON parse errors
        }

        throw new Error(errorMessage);
      }

      // Check if response is streaming
      const contentType = response.headers.get('content-type');
      if (contentType?.includes('text/event-stream')) {
        // Handle streaming response
        const messageId = `assistant-${Date.now()}`;
        dispatch({ type: 'START_STREAMING', payload: { messageId } });

        const reader = response.body?.getReader();
        const decoder = new TextDecoder();

        if (!reader) {
          throw new Error('Response body is not readable');
        }

        try {
          let buffer = '';

          while (true) {
            const { done, value } = await reader.read();

            if (done) {
              if (buffer.trim()) {
                // Process any remaining data in buffer
                const lines = buffer.split('\n');
                for (const line of lines) {
                  if (line.startsWith('data: ')) {
                    try {
                      const chunk: StreamChunk = JSON.parse(line.substring(6));
                      if (chunk.done) {
                        dispatch({
                          type: 'COMPLETE_STREAM',
                          payload: {
                            sources: chunk.sources || [],
                            confidence: chunk.confidence || 0,
                            session_id: chunk.session_id || state.sessionId || '',
                            tokens_used: chunk.tokens_used || { input_tokens: 0, output_tokens: 0, total_tokens: 0 }
                          }
                        });
                      } else if (chunk.chunk) {
                        dispatch({ type: 'APPEND_STREAM_CHUNK', payload: { chunk: chunk.chunk } });
                      }
                    } catch (e) {
                      console.error('Failed to parse SSE chunk:', e);
                    }
                  }
                }
              }
              break;
            }

            // Decode and add to buffer
            buffer += decoder.decode(value, { stream: true });

            // Process complete SSE messages (separated by \n\n)
            const messages = buffer.split('\n\n');
            buffer = messages.pop() || ''; // Keep incomplete message in buffer

            for (const message of messages) {
              if (!message.trim()) continue;

              const lines = message.split('\n');
              for (const line of lines) {
                if (line.startsWith('data: ')) {
                  try {
                    const chunk: StreamChunk = JSON.parse(line.substring(6));

                    if (chunk.done) {
                      // Final chunk with metadata
                      dispatch({
                        type: 'COMPLETE_STREAM',
                        payload: {
                          sources: chunk.sources || [],
                          confidence: chunk.confidence || 0,
                          session_id: chunk.session_id || state.sessionId || '',
                          tokens_used: chunk.tokens_used || { input_tokens: 0, output_tokens: 0, total_tokens: 0 }
                        }
                      });
                    } else if (chunk.chunk) {
                      // Text chunk
                      dispatch({ type: 'APPEND_STREAM_CHUNK', payload: { chunk: chunk.chunk } });
                    }
                  } catch (e) {
                    console.error('Failed to parse SSE chunk:', e);
                  }
                }
              }
            }
          }
        } catch (streamError) {
          console.error('Stream reading error:', streamError);
          dispatch({
            type: 'INTERRUPT_STREAM',
            payload: { error: 'Stream was interrupted. Click retry to get the complete response.' }
          });
        }
      } else {
        // Fallback to non-streaming JSON response
        const data: QueryResponse = await response.json();
        dispatch({ type: 'ADD_ASSISTANT_MESSAGE', payload: data });
      }
    } catch (error) {
      const errorMessage = error instanceof Error
        ? error.message
        : 'Failed to get response. Please try again.';

      dispatch({ type: 'SET_ERROR', payload: errorMessage });
    }
  }, [state.sessionId, state.moduleFilter, difficultyFilter]);

  /**
   * Clear chat history (UI + backend)
   */
  const handleClearHistory = useCallback(async () => {
    if (!state.sessionId) {
      // No session to clear
      dispatch({ type: 'CLEAR_HISTORY' });
      if (typeof window !== 'undefined') {
        localStorage.removeItem(SESSION_STORAGE_KEY);
      }
      return;
    }

    try {
      // Call backend DELETE endpoint
      await fetch(`${API_BASE_URL}/api/v1/chat/sessions/${state.sessionId}`, {
        method: 'DELETE'
      });

      // Clear local state
      dispatch({ type: 'CLEAR_HISTORY' });
      if (typeof window !== 'undefined') {
        localStorage.removeItem(SESSION_STORAGE_KEY);
      }
    } catch (error) {
      console.error('Failed to clear history:', error);
      // Still clear local state even if backend fails
      dispatch({ type: 'CLEAR_HISTORY' });
      if (typeof window !== 'undefined') {
        localStorage.removeItem(SESSION_STORAGE_KEY);
      }
    }
  }, [state.sessionId]);

  /**
   * Handle module filter change
   */
  const handleModuleFilterChange = useCallback((module: number | null) => {
    dispatch({ type: 'SET_MODULE_FILTER', payload: module });
  }, []);

  /**
   * Retry after error
   */
  const handleRetry = useCallback(() => {
    dispatch({ type: 'SET_ERROR', payload: null });
    // Re-submit last user message if it exists
    const lastUserMessage = [...state.messages]
      .reverse()
      .find(m => m.role === 'user');

    if (lastUserMessage) {
      handleSubmit(lastUserMessage.content);
    }
  }, [state.messages, handleSubmit]);

  /**
   * Retry after stream interruption
   */
  const handleRetryStream = useCallback(() => {
    dispatch({ type: 'RETRY_STREAM' });

    // Remove last incomplete assistant message
    const lastMessage = state.messages[state.messages.length - 1];
    if (lastMessage && lastMessage.role === 'assistant') {
      // Remove from messages by dispatching clear and re-adding previous messages
      const previousMessages = state.messages.slice(0, -1);
      dispatch({ type: 'CLEAR_HISTORY' });
      // Restore session and previous messages
      setTimeout(() => {
        dispatch({ type: 'SET_SESSION_ID', payload: state.sessionId || '' });
      }, 0);
    }

    // Re-submit last user message
    const lastUserMessage = [...state.messages]
      .reverse()
      .find(m => m.role === 'user');

    if (lastUserMessage) {
      handleSubmit(lastUserMessage.content);
    }
  }, [state.messages, state.sessionId, handleSubmit]);

  /**
   * Countdown timer for rate limit
   */
  useEffect(() => {
    if (!state.rateLimitState.isRateLimited || !state.rateLimitState.resetTime) {
      return;
    }

    const intervalId = setInterval(() => {
      const now = Date.now();
      const resetTime = state.rateLimitState.resetTime?.getTime() || 0;
      const remainingSeconds = Math.max(0, Math.floor((resetTime - now) / 1000));

      if (remainingSeconds <= 0) {
        // Rate limit has expired, reset state
        dispatch({
          type: 'SET_RATE_LIMIT',
          payload: {
            isRateLimited: false,
            retryAfter: 0,
            resetTime: null
          }
        });
      } else {
        // Update remaining time
        dispatch({
          type: 'SET_RATE_LIMIT',
          payload: {
            isRateLimited: true,
            retryAfter: remainingSeconds,
            resetTime: state.rateLimitState.resetTime
          }
        });
      }
    }, 1000); // Update every second

    return () => clearInterval(intervalId);
  }, [state.rateLimitState.isRateLimited, state.rateLimitState.resetTime]);

  /**
   * Keyboard navigation: Escape to close (if embedded in modal)
   */
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        // Dispatch custom event for parent to handle
        const event = new CustomEvent('chatbot-close');
        window.dispatchEvent(event);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, []);

  // SSR compatibility: Don't render on server
  if (typeof window === 'undefined') {
    return <div>Loading chatbot...</div>;
  }

  return (
    <div
      className={styles.chatbotWidget}
      role="region"
      aria-label="Robotics textbook chatbot"
    >
      {/* Header */}
      <div className={styles.chatbotHeader}>
        <h2 className={styles.chatbotTitle}>
          Ask About Robotics
        </h2>

        {state.messages.length > 0 && (
          <button
            onClick={handleClearHistory}
            className={styles.clearButton}
            aria-label="Clear chat history"
            type="button"
          >
            Clear History
          </button>
        )}
      </div>

      {/* Module Filter */}
      <ModuleFilter
        selectedModule={state.moduleFilter}
        onModuleChange={handleModuleFilterChange}
      />

      {/* Rate limit banner */}
      {state.rateLimitState.isRateLimited && (
        <div
          className={styles.rateLimitBanner}
          role="alert"
          aria-live="polite"
        >
          <svg
            width="20"
            height="20"
            viewBox="0 0 20 20"
            fill="currentColor"
            aria-hidden="true"
          >
            <path d="M10 0C4.48 0 0 4.48 0 10s4.48 10 10 10 10-4.48 10-10S15.52 0 10 0zm0 18c-4.41 0-8-3.59-8-8s3.59-8 8-8 8 3.59 8 8-3.59 8-8 8zm.5-13H9v6l5.25 3.15.75-1.23-4.5-2.67z" />
          </svg>
          <div>
            <div>
              <strong>Rate limit exceeded.</strong> Please try again in{' '}
              <strong>
                {Math.floor(state.rateLimitState.retryAfter / 60)}m{' '}
                {state.rateLimitState.retryAfter % 60}s
              </strong>
            </div>
            {!isAuthenticated && (
              <div style={{ marginTop: '8px', fontSize: '0.9em' }}>
                💡 <Link to="/signup" style={{ fontWeight: 'bold', textDecoration: 'underline' }}>
                  Create a free account
                </Link> to get 50 queries/hour (5x more than anonymous users)
              </div>
            )}
          </div>
        </div>
      )}

      {/* Error banner */}
      {state.error && (
        <div
          className={styles.errorBanner}
          role="alert"
          aria-live="assertive"
        >
          <svg
            width="20"
            height="20"
            viewBox="0 0 20 20"
            fill="currentColor"
            aria-hidden="true"
          >
            <path d="M10 0C4.48 0 0 4.48 0 10s4.48 10 10 10 10-4.48 10-10S15.52 0 10 0zm1 15H9v-2h2v2zm0-4H9V5h2v6z" />
          </svg>
          <span>{state.error}</span>
          <button
            onClick={handleRetry}
            className={styles.retryButton}
            type="button"
          >
            Retry
          </button>
        </div>
      )}

      {/* Stream interruption banner */}
      {state.streamError && (
        <div
          className={styles.errorBanner}
          role="alert"
          aria-live="assertive"
        >
          <svg
            width="20"
            height="20"
            viewBox="0 0 20 20"
            fill="currentColor"
            aria-hidden="true"
          >
            <path d="M10 0C4.48 0 0 4.48 0 10s4.48 10 10 10 10-4.48 10-10S15.52 0 10 0zm1 15H9v-2h2v2zm0-4H9V5h2v6z" />
          </svg>
          <span>{state.streamError}</span>
          <button
            onClick={handleRetryStream}
            className={styles.retryButton}
            type="button"
            aria-label="Retry interrupted stream"
          >
            Retry
          </button>
        </div>
      )}

      {/* Message list */}
      <MessageList
        messages={state.messages}
        isLoading={state.isLoading}
      />

      {/* Streaming indicator */}
      {state.isStreaming && (
        <LoadingState isStreaming={true} />
      )}

      {/* Input form */}
      <ChatInput
        onSubmit={handleSubmit}
        isLoading={state.isLoading || state.rateLimitState.isRateLimited}
      />
    </div>
  );
}
