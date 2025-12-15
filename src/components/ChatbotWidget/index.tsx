import React, { useState, useEffect, useCallback, useRef } from 'react';
import clsx from 'clsx';
import {
  ChatbotWidgetProps,
  ChatMessage,
  ChatState,
  FilterParams,
  QueryRequest,
  ApiResponse,
  StreamingEvent,
  RateLimitResponse,
  RateLimitState,
} from './types';
import { ChatInput } from './ChatInput';
import { MessageList } from './MessageList';
import { SourceCitations } from './SourceCitations';
import { ConfidenceIndicator } from './ConfidenceIndicator';
import { ModuleFilter } from './ModuleFilter';

// Helper to check if response is a rate limit error
function isRateLimitResponse(response: any): response is RateLimitResponse {
  return response && response.error === 'Rate limit exceeded' && 'resetAfter' in response;
}

// Helper to format time remaining
function formatTimeRemaining(seconds: number): string {
  if (seconds <= 0) return 'now';
  if (seconds < 60) return `${seconds} second${seconds !== 1 ? 's' : ''}`;
  const minutes = Math.floor(seconds / 60);
  const remainingSeconds = seconds % 60;
  if (minutes < 60) {
    if (remainingSeconds > 0) {
      return `${minutes} minute${minutes !== 1 ? 's' : ''} ${remainingSeconds} second${remainingSeconds !== 1 ? 's' : ''}`;
    }
    return `${minutes} minute${minutes !== 1 ? 's' : ''}`;
  }
  const hours = Math.floor(minutes / 60);
  const remainingMinutes = minutes % 60;
  return `${hours} hour${hours !== 1 ? 's' : ''} ${remainingMinutes} minute${remainingMinutes !== 1 ? 's' : ''}`;
}

export const ChatbotWidget: React.FC<ChatbotWidgetProps> = ({
  apiUrl = 'http://localhost:8000',
  title = 'Ask a Question',
  placeholder = 'Ask about robotics...',
  maxMessages = 50,
  showSuggestions = true,
  showTokenUsage = true,
  allowFiltering = true,
  className,
  onMessageSent,
  onResponseReceived,
  onError,
}) => {
  const [chatState, setChatState] = useState<ChatState>({
    messages: [],
    isLoading: false,
    isStreaming: false,
    sessionId: null,
    error: null,
    suggestedQuestions: [],
    rateLimit: null,
  });

  const [filters, setFilters] = useState<FilterParams>({});
  const [isMinimized, setIsMinimized] = useState(false);
  const [rateLimitCountdown, setRateLimitCountdown] = useState<number>(0);
  const eventSourceRef = useRef<EventSource | null>(null);
  const abortControllerRef = useRef<AbortController | null>(null);
  const countdownIntervalRef = useRef<NodeJS.Timeout | null>(null);

  // Load session from localStorage on mount
  useEffect(() => {
    const savedSessionId = localStorage.getItem('chatbot-session-id');
    if (savedSessionId) {
      setChatState((prev) => ({ ...prev, sessionId: savedSessionId }));
      // Load previous messages if needed
      loadChatHistory(savedSessionId);
    }
  }, []);

  // Save session to localStorage when it changes
  useEffect(() => {
    if (chatState.sessionId) {
      localStorage.setItem('chatbot-session-id', chatState.sessionId);
    }
  }, [chatState.sessionId]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (eventSourceRef.current) {
        eventSourceRef.current.close();
      }
      if (abortControllerRef.current) {
        abortControllerRef.current.abort();
      }
      if (countdownIntervalRef.current) {
        clearInterval(countdownIntervalRef.current);
      }
    };
  }, []);

  // Rate limit countdown timer
  useEffect(() => {
    if (chatState.rateLimit?.isLimited && chatState.rateLimit.resetTime) {
      // Calculate initial countdown
      const resetTime = new Date(chatState.rateLimit.resetTime).getTime();
      const now = Date.now();
      const initialSeconds = Math.max(0, Math.ceil((resetTime - now) / 1000));
      setRateLimitCountdown(initialSeconds);

      // Start countdown interval
      countdownIntervalRef.current = setInterval(() => {
        const remaining = Math.max(0, Math.ceil((resetTime - Date.now()) / 1000));
        setRateLimitCountdown(remaining);

        // Clear rate limit when countdown reaches 0
        if (remaining <= 0) {
          if (countdownIntervalRef.current) {
            clearInterval(countdownIntervalRef.current);
          }
          setChatState((prev) => ({
            ...prev,
            rateLimit: null,
            error: null,
          }));
        }
      }, 1000);

      return () => {
        if (countdownIntervalRef.current) {
          clearInterval(countdownIntervalRef.current);
        }
      };
    }
  }, [chatState.rateLimit?.isLimited, chatState.rateLimit?.resetTime]);

  const loadChatHistory = async (sessionId: string) => {
    try {
      const response = await fetch(`${apiUrl}/api/v1/chat/sessions/${sessionId}`);
      if (response.ok) {
        const data = await response.json();
        setChatState((prev) => ({
          ...prev,
          messages: data.messages.map(formatMessage),
        }));
      }
    } catch (error) {
      console.error('Failed to load chat history:', error);
    }
  };

  const formatMessage = (msg: any): ChatMessage => ({
    id: msg.id,
    role: msg.role,
    content: msg.content,
    timestamp: new Date(msg.timestamp),
    sources: msg.sources,
    confidence: msg.confidence,
    tokenUsage: msg.tokenUsage,
    filterMessage: msg.filterMessage,
    warning: msg.warning,
    suggestedRephrase: msg.suggestedRephrase,
    suggestedTopics: msg.suggestedTopics,
    suggestedQueries: msg.suggestedQueries,
    error: msg.error,
  });

  const handleSendMessage = useCallback(
    async (query: string) => {
      if (!query.trim()) return;

      // Add user message immediately
      const userMessage: ChatMessage = {
        id: `user-${Date.now()}`,
        role: 'user',
        content: query,
        timestamp: new Date(),
      };

      setChatState((prev) => ({
        ...prev,
        messages: [...prev.messages, userMessage],
        isLoading: true,
        error: null,
      }));

      onMessageSent?.(query);

      try {
        // Check if client wants streaming
        const acceptStreaming = true; // Could be a prop to control this

        if (acceptStreaming) {
          await sendStreamingQuery(query);
        } else {
          await sendNormalQuery(query);
        }
      } catch (error) {
        console.error('Error sending message:', error);
        handleError(error);
      }
    },
    [filters, chatState.sessionId, apiUrl, onMessageSent, onError]
  );

  const sendNormalQuery = async (query: string) => {
    const request: QueryRequest = {
      query,
      sessionId: chatState.sessionId || undefined,
      filters: Object.keys(filters).length > 0 ? filters : undefined,
    };

    const response = await fetch(`${apiUrl}/api/v1/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    const data: ApiResponse = await response.json();

    // Handle rate limit error specially
    if (response.status === 429 && isRateLimitResponse(data)) {
      handleRateLimitError(data);
      return;
    }

    if (!response.ok) {
      throw new Error(data.error || 'Failed to send query');
    }

    handleResponse(data);
  };

  const handleRateLimitError = (data: RateLimitResponse) => {
    const resetTime = new Date(Date.now() + data.resetAfter * 1000);

    setChatState((prev) => ({
      ...prev,
      isLoading: false,
      isStreaming: false,
      error: `Rate limit exceeded. You can make ${data.limit} requests per hour.`,
      rateLimit: {
        isLimited: true,
        limit: data.limit,
        resetAfter: data.resetAfter,
        resetTime: resetTime,
      },
    }));

    onError?.(`Rate limit exceeded. Try again in ${data.retryAfter}.`);
  };

  const sendStreamingQuery = async (query: string) => {
    // Cancel any existing request
    if (abortControllerRef.current) {
      abortControllerRef.current.abort();
    }

    abortControllerRef.current = new AbortController();

    const request: QueryRequest = {
      query,
      sessionId: chatState.sessionId || undefined,
      filters: Object.keys(filters).length > 0 ? filters : undefined,
    };

    // Create streaming response
    const response = await fetch(`${apiUrl}/api/v1/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        Accept: 'text/event-stream',
      },
      body: JSON.stringify(request),
      signal: abortControllerRef.current.signal,
    });

    // Handle rate limit error
    if (response.status === 429) {
      const data = await response.json();
      if (isRateLimitResponse(data)) {
        handleRateLimitError(data);
        return;
      }
    }

    if (!response.ok) {
      throw new Error('Failed to start streaming');
    }

    // Start streaming
    const reader = response.body?.getReader();
    const decoder = new TextDecoder();

    if (!reader) {
      throw new Error('Response body is not readable');
    }

    let assistantMessage: ChatMessage | null = null;
    setChatState((prev) => ({ ...prev, isStreaming: true }));

    try {
      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        const chunk = decoder.decode(value, { stream: true });
        const lines = chunk.split('\n');

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            try {
              const data: StreamingEvent = JSON.parse(line.slice(6));
              handleStreamingEvent(data, (msg) => {
                assistantMessage = msg;
                setChatState((prev) => ({
                  ...prev,
                  messages: [...prev.messages, msg],
                }));
              });
            } catch (e) {
              console.error('Failed to parse SSE data:', e);
            }
          }
        }
      }
    } finally {
      reader.releaseLock();
      setChatState((prev) => ({ ...prev, isStreaming: false, isLoading: false }));
    }
  };

  const handleStreamingEvent = (event: StreamingEvent, onMessage: (msg: ChatMessage) => void) => {
    switch (event.type) {
      case 'start':
        setChatState((prev) => ({ ...prev, sessionId: event.sessionId }));
        break;

      case 'chunk':
        // Update streaming message
        setChatState((prev) => {
          const messages = [...prev.messages];
          const lastMessage = messages[messages.length - 1];
          if (lastMessage && lastMessage.role === 'assistant' && prev.isStreaming) {
            messages[messages.length - 1] = {
              ...lastMessage,
              content: lastMessage.content + event.chunk,
            };
          } else {
            messages.push({
              id: `assistant-${Date.now()}`,
              role: 'assistant',
              content: event.chunk,
              timestamp: new Date(),
            });
          }
          return { ...prev, messages };
        });
        break;

      case 'end':
        const message: ChatMessage = {
          id: `assistant-${Date.now()}`,
          role: 'assistant',
          content: event.answer,
          timestamp: new Date(),
          sources: event.sources,
          confidence: event.confidence,
          tokenUsage: event.tokenUsage,
          filterMessage: event.filterMessage,
        };
        onMessage(message);
        onResponseReceived?.(event);
        break;

      case 'off_topic':
        const offTopicMessage: ChatMessage = {
          id: `assistant-${Date.now()}`,
          role: 'assistant',
          content: event.message,
          timestamp: new Date(),
          suggestedTopics: event.suggestedTopics,
          suggestedQueries: event.suggestedQueries,
        };
        onMessage(offTopicMessage);
        break;

      case 'error':
        throw new Error(event.error);
    }
  };

  const handleResponse = (data: ApiResponse) => {
    const message: ChatMessage = {
      id: `assistant-${Date.now()}`,
      role: 'assistant',
      content: 'answer' in data ? data.answer : data.message,
      timestamp: new Date(),
      sources: 'sources' in data ? data.sources : undefined,
      confidence: 'confidence' in data ? data.confidence : undefined,
      tokenUsage: 'tokenUsage' in data ? data.tokenUsage : undefined,
      filterMessage: 'filterMessage' in data ? data.filterMessage : undefined,
      warning: 'warning' in data ? data.warning : undefined,
      suggestedRephrase: 'suggestedRephrase' in data ? data.suggestedRephrase : undefined,
      suggestedTopics: 'suggestedTopics' in data ? data.suggestedTopics : undefined,
      suggestedQueries: 'suggestedQueries' in data ? data.suggestedQueries : undefined,
    };

    setChatState((prev) => ({
      ...prev,
      messages: [...prev.messages, message],
      isLoading: false,
      sessionId: 'sessionId' in data ? data.sessionId : prev.sessionId,
    }));

    onResponseReceived?.(data);
  };

  const handleError = (error: any) => {
    const errorMessage = error?.message || 'An unexpected error occurred';
    setChatState((prev) => ({
      ...prev,
      isLoading: false,
      isStreaming: false,
      error: errorMessage,
    }));
    onError?.(errorMessage);
  };

  const handleClearHistory = async () => {
    if (chatState.sessionId) {
      try {
        await fetch(`${apiUrl}/api/v1/chat/sessions/${chatState.sessionId}`, {
          method: 'DELETE',
        });
      } catch (error) {
        console.error('Failed to clear session:', error);
      }
    }

    localStorage.removeItem('chatbot-session-id');
    setChatState({
      messages: [],
      isLoading: false,
      isStreaming: false,
      sessionId: null,
      error: null,
      suggestedQuestions: [],
    });
  };

  const handleSuggestedQuery = (query: string) => {
    handleSendMessage(query);
  };

  const handleFilterChange = (newFilters: FilterParams) => {
    setFilters(newFilters);
  };

  return (
    <div className={clsx('chatbot-widget', className, { minimized: isMinimized })}>
      <div className="widget-header">
        <h3 className="widget-title">{title}</h3>
        <div className="header-controls">
          {showTokenUsage && chatState.messages.length > 0 && (
            <div className="token-info">
              Total:{' '}
              {chatState.messages.reduce((sum, msg) => sum + (msg.tokenUsage?.totalTokens || 0), 0)}{' '}
              tokens
            </div>
          )}
          <button
            className="minimize-button"
            onClick={() => setIsMinimized(!isMinimized)}
            aria-label={isMinimized ? 'Expand chat' : 'Minimize chat'}
          >
            {isMinimized ? '▲' : '▼'}
          </button>
        </div>
      </div>

      {!isMinimized && (
        <>
          {allowFiltering && (
            <div className="widget-filter">
              <ModuleFilter onFilterChange={handleFilterChange} filters={filters} />
            </div>
          )}

          <MessageList
            messages={chatState.messages}
            isStreaming={chatState.isStreaming}
            maxMessages={maxMessages}
          />

          {chatState.error && (
            <div
              className={clsx('error-banner', {
                'rate-limit-banner': chatState.rateLimit?.isLimited,
              })}
            >
              <span className="error-icon">{chatState.rateLimit?.isLimited ? '⏱️' : '⚠️'}</span>
              <div className="error-content">
                <span className="error-message">{chatState.error}</span>
                {chatState.rateLimit?.isLimited && rateLimitCountdown > 0 && (
                  <span className="rate-limit-countdown">
                    Try again in: <strong>{formatTimeRemaining(rateLimitCountdown)}</strong>
                  </span>
                )}
              </div>
              {!chatState.rateLimit?.isLimited && (
                <button
                  className="retry-button"
                  onClick={() => setChatState((prev) => ({ ...prev, error: null }))}
                >
                  Dismiss
                </button>
              )}
            </div>
          )}

          {showSuggestions && chatState.messages.length === 0 && !chatState.isLoading && (
            <div className="suggested-questions">
              <h4>Try asking:</h4>
              <div className="suggestion-chips">
                {[
                  'What is inverse kinematics?',
                  'How do sensors work in robots?',
                  'What is PID control?',
                ].map((question, idx) => (
                  <button
                    key={idx}
                    className="suggestion-chip"
                    onClick={() => handleSuggestedQuery(question)}
                  >
                    {question}
                  </button>
                ))}
              </div>
            </div>
          )}

          <ChatInput
            onSendMessage={handleSendMessage}
            onClearHistory={handleClearHistory}
            disabled={
              chatState.isLoading || chatState.isStreaming || chatState.rateLimit?.isLimited
            }
            placeholder={
              chatState.rateLimit?.isLimited
                ? `Rate limited. Try again in ${formatTimeRemaining(rateLimitCountdown)}`
                : placeholder
            }
          />
        </>
      )}

      <style jsx>{`
        .chatbot-widget {
          display: flex;
          flex-direction: column;
          height: 600px;
          width: 100%;
          max-width: 500px;
          background-color: var(--ifm-background-color);
          border: 1px solid var(--ifm-color-emphasis-200);
          border-radius: 12px;
          overflow: hidden;
          box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
          transition: all 0.3s ease;
        }

        .chatbot-widget.minimized {
          height: 60px;
        }

        .widget-header {
          display: flex;
          justify-content: space-between;
          align-items: center;
          padding: 1rem;
          background-color: var(--ifm-color-primary);
          color: white;
          border-bottom: 1px solid var(--ifm-color-emphasis-200);
        }

        .widget-title {
          margin: 0;
          font-size: 1.1rem;
          font-weight: 600;
        }

        .header-controls {
          display: flex;
          align-items: center;
          gap: 1rem;
        }

        .token-info {
          font-size: 0.75rem;
          opacity: 0.8;
        }

        .minimize-button {
          background: none;
          border: none;
          color: white;
          cursor: pointer;
          font-size: 1.2rem;
          padding: 0.25rem;
          border-radius: 4px;
          transition: background-color 0.2s;
        }

        .minimize-button:hover {
          background-color: rgba(255, 255, 255, 0.1);
        }

        .widget-filter {
          padding: 0.75rem 1rem;
          border-bottom: 1px solid var(--ifm-color-emphasis-200);
          background-color: var(--ifm-background-surface-color);
        }

        .error-banner {
          display: flex;
          align-items: center;
          gap: 0.75rem;
          padding: 0.75rem 1rem;
          background-color: var(--ifm-color-danger-contrast-background);
          color: var(--ifm-color-danger);
          border-top: 1px solid var(--ifm-color-emphasis-200);
        }

        .error-icon {
          font-size: 1.2rem;
        }

        .retry-button {
          margin-left: auto;
          padding: 0.25rem 0.75rem;
          background: none;
          border: 1px solid var(--ifm-color-danger);
          color: var(--ifm-color-danger);
          border-radius: 4px;
          cursor: pointer;
          font-size: 0.875rem;
          transition: all 0.2s;
        }

        .retry-button:hover {
          background-color: var(--ifm-color-danger);
          color: white;
        }

        .rate-limit-banner {
          background-color: var(--ifm-color-warning-contrast-background);
          color: var(--ifm-color-warning-dark);
          flex-direction: column;
          align-items: flex-start;
        }

        .rate-limit-banner .error-icon {
          font-size: 1.5rem;
        }

        .error-content {
          display: flex;
          flex-direction: column;
          gap: 0.25rem;
          flex: 1;
        }

        .error-message {
          font-weight: 500;
        }

        .rate-limit-countdown {
          font-size: 0.875rem;
          color: var(--ifm-color-warning-darker);
        }

        .rate-limit-countdown strong {
          font-family: monospace;
          background-color: var(--ifm-color-warning-contrast-background);
          padding: 0.125rem 0.375rem;
          border-radius: 4px;
        }

        .suggested-questions {
          padding: 1rem;
          border-top: 1px solid var(--ifm-color-emphasis-200);
        }

        .suggested-questions h4 {
          margin: 0 0 0.75rem 0;
          font-size: 0.9rem;
          color: var(--ifm-color-emphasis-700);
        }

        .suggestion-chips {
          display: flex;
          flex-wrap: wrap;
          gap: 0.5rem;
        }

        .suggestion-chip {
          padding: 0.5rem 0.75rem;
          background-color: var(--ifm-color-emphasis-100);
          border: 1px solid var(--ifm-color-emphasis-200);
          border-radius: 20px;
          font-size: 0.875rem;
          color: var(--ifm-color-emphasis-800);
          cursor: pointer;
          transition: all 0.2s;
        }

        .suggestion-chip:hover {
          background-color: var(--ifm-color-primary);
          color: white;
          border-color: var(--ifm-color-primary);
        }

        /* Mobile adjustments */
        @media (max-width: 768px) {
          .chatbot-widget {
            height: 100vh;
            max-width: 100%;
            border-radius: 0;
          }

          .chatbot-widget.minimized {
            height: 120px;
          }

          .widget-header {
            padding: 0.75rem;
          }

          .token-info {
            display: none;
          }
        }
      `}</style>
    </div>
  );
};
