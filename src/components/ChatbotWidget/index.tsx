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
import styles from './ChatbotWidget.module.css';
import type {
  ChatState,
  ChatAction,
  Message,
  QueryRequest,
  QueryResponse,
  ErrorResponse
} from './types';
import ChatInput from './ChatInput';
import MessageList from './MessageList';

// Environment configuration
const API_BASE_URL = typeof window !== 'undefined'
  ? (window as any).CHATBOT_API_URL || 'http://localhost:8000'
  : 'http://localhost:8000';

const SESSION_STORAGE_KEY = 'chatbot-session-id';

/**
 * Chat state reducer
 * Manages messages, session, loading, and error states
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
        error: null
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
        error: null
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
        isLoading: false
      };

    case 'SET_SESSION_ID':
      return {
        ...state,
        sessionId: action.payload
      };

    case 'CLEAR_HISTORY':
      return {
        messages: [],
        sessionId: null,
        isLoading: false,
        error: null
      };

    default:
      return state;
  }
}

/**
 * Initial state factory
 */
function getInitialState(): ChatState {
  // SSR compatibility check
  if (typeof window === 'undefined') {
    return {
      messages: [],
      sessionId: null,
      isLoading: false,
      error: null
    };
  }

  // Restore session from localStorage
  const storedSessionId = localStorage.getItem(SESSION_STORAGE_KEY);

  return {
    messages: [],
    sessionId: storedSessionId,
    isLoading: false,
    error: null
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
   * Submit query to backend
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

      // Apply filters if provided
      if (moduleFilter || difficultyFilter) {
        requestBody.filters = {
          module: moduleFilter,
          difficulty: difficultyFilter
        };
      }

      const response = await fetch(`${API_BASE_URL}/api/v1/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(requestBody)
      });

      if (!response.ok) {
        // Handle HTTP errors
        let errorMessage = `Server error: ${response.status}`;

        try {
          const errorData: ErrorResponse = await response.json();
          errorMessage = errorData.detail || errorMessage;
        } catch {
          // Ignore JSON parse errors
        }

        throw new Error(errorMessage);
      }

      const data: QueryResponse = await response.json();

      // Add assistant response to UI
      dispatch({ type: 'ADD_ASSISTANT_MESSAGE', payload: data });
    } catch (error) {
      const errorMessage = error instanceof Error
        ? error.message
        : 'Failed to get response. Please try again.';

      dispatch({ type: 'SET_ERROR', payload: errorMessage });
    }
  }, [state.sessionId, moduleFilter, difficultyFilter]);

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

      {/* Message list */}
      <MessageList
        messages={state.messages}
        isLoading={state.isLoading}
      />

      {/* Input form */}
      <ChatInput
        onSubmit={handleSubmit}
        isLoading={state.isLoading}
      />
    </div>
  );
}
