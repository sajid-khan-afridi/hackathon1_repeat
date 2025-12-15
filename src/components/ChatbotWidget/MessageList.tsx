import React, { useRef, useEffect } from 'react';
import clsx from 'clsx';
import { ChatMessage, MessageListProps } from './types';
import { SourceCitations } from './SourceCitations';
import { ConfidenceIndicator } from './ConfidenceIndicator';

export const MessageList: React.FC<MessageListProps> = ({
  messages,
  isStreaming = false,
  maxMessages = 50,
  className,
}) => {
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages, isStreaming]);

  // Limit messages to maxMessages
  const displayedMessages = messages.slice(-maxMessages);

  const formatTime = (date: Date) => {
    return date.toLocaleTimeString(undefined, {
      hour: '2-digit',
      minute: '2-digit',
    });
  };

  const renderMessage = (message: ChatMessage, index: number) => {
    const isUser = message.role === 'user';
    const showDivider = index > 0 && displayedMessages[index - 1].role !== message.role;

    return (
      <React.Fragment key={message.id}>
        {showDivider && <div className="message-divider" />}
        <div
          className={clsx('message', {
            'user-message': isUser,
            'assistant-message': !isUser,
          })}
        >
          <div className="message-content">
            {message.error ? (
              <div className="error-message">
                <strong>Error:</strong> {message.error}
              </div>
            ) : (
              <>
                <div className="message-text">{message.content}</div>
                {message.suggestedTopics && (
                  <div className="suggested-topics">
                    <p>I can help you with these topics:</p>
                    <div className="topic-chips">
                      {message.suggestedTopics.map((topic, idx) => (
                        <span key={idx} className="topic-chip">
                          {topic}
                        </span>
                      ))}
                    </div>
                  </div>
                )}
                {message.suggestedQueries && (
                  <div className="suggested-queries">
                    <p>Try asking:</p>
                    <ul>
                      {message.suggestedQueries.slice(0, 5).map((query, idx) => (
                        <li key={idx}>{query}</li>
                      ))}
                    </ul>
                  </div>
                )}
                {message.warning && (
                  <div className="warning-message">
                    <span className="warning-icon">⚠️</span>
                    {message.warning}
                    {message.suggestedRephrase && (
                      <p className="suggestion">Suggestion: {message.suggestedRephrase}</p>
                    )}
                  </div>
                )}
              </>
            )}
          </div>

          <div className="message-meta">
            <span className="message-time">{formatTime(message.timestamp)}</span>
            {!isUser && message.confidence && (
              <ConfidenceIndicator confidence={message.confidence} size="small" />
            )}
            {message.tokenUsage && (
              <span className="token-usage" title="Tokens used">
                📊 {message.tokenUsage.totalTokens}
              </span>
            )}
          </div>

          {!isUser && message.sources && message.sources.length > 0 && (
            <SourceCitations sources={message.sources} />
          )}

          {message.filterMessage && (
            <div className="filter-message">
              <span className="filter-icon">🔍</span>
              {message.filterMessage}
            </div>
          )}
        </div>
      </React.Fragment>
    );
  };

  return (
    <div
      ref={containerRef}
      className={clsx('message-list', className)}
      aria-label="Chat messages"
      aria-live="polite"
      aria-atomic="false"
    >
      {displayedMessages.length === 0 && !isStreaming && (
        <div className="empty-state">
          <div className="empty-state-icon">💬</div>
          <h3>Ask me anything about robotics!</h3>
          <p>
            I can help you with topics like kinematics, control systems, sensors, and more. Try
            asking about a specific chapter or concept.
          </p>
        </div>
      )}

      {displayedMessages.map(renderMessage)}

      {isStreaming && (
        <div className="message assistant-message streaming">
          <div className="message-content">
            <div className="streaming-indicator">
              <span className="dot"></span>
              <span className="dot"></span>
              <span className="dot"></span>
            </div>
          </div>
        </div>
      )}

      <div ref={messagesEndRef} />

      <style jsx>{`
        .message-list {
          flex: 1;
          overflow-y: auto;
          padding: 1rem;
          scroll-behavior: smooth;
        }

        .message {
          display: flex;
          flex-direction: column;
          gap: 0.5rem;
          margin-bottom: 1.5rem;
          max-width: 90%;
        }

        .user-message {
          align-self: flex-end;
          align-items: flex-end;
        }

        .assistant-message {
          align-self: flex-start;
          align-items: flex-start;
        }

        .message-divider {
          height: 1px;
          background-color: var(--ifm-color-emphasis-200);
          margin: 1rem 0;
        }

        .message-content {
          background-color: var(--ifm-background-surface-color);
          padding: 0.75rem 1rem;
          border-radius: 12px;
          box-shadow: 0 1px 2px rgba(0, 0, 0, 0.05);
        }

        .user-message .message-content {
          background-color: var(--ifm-color-primary);
          color: white;
        }

        .message-text {
          white-space: pre-wrap;
          word-wrap: break-word;
          line-height: 1.5;
        }

        .error-message {
          color: var(--ifm-color-danger);
          background-color: var(--ifm-color-danger-contrast-background);
          padding: 0.5rem;
          border-radius: 6px;
          border: 1px solid var(--ifm-color-danger-contrast-foreground);
        }

        .warning-message {
          margin-top: 0.5rem;
          padding: 0.5rem;
          background-color: var(--ifm-color-warning-contrast-background);
          border-radius: 6px;
          border-left: 3px solid var(--ifm-color-warning);
        }

        .warning-icon {
          margin-right: 0.5rem;
        }

        .suggestion {
          margin: 0.5rem 0 0 0;
          font-size: 0.9em;
          font-style: italic;
        }

        .suggested-topics,
        .suggested-queries {
          margin-top: 1rem;
          padding: 0.75rem;
          background-color: var(--ifm-color-emphasis-100);
          border-radius: 6px;
        }

        .suggested-topics p,
        .suggested-queries p {
          margin: 0 0 0.5rem 0;
          font-weight: 600;
          color: var(--ifm-color-emphasis-800);
        }

        .topic-chips {
          display: flex;
          flex-wrap: wrap;
          gap: 0.5rem;
        }

        .topic-chip {
          padding: 0.25rem 0.75rem;
          background-color: var(--ifm-color-primary);
          color: white;
          border-radius: 16px;
          font-size: 0.875rem;
          cursor: pointer;
          transition: opacity 0.2s;
        }

        .topic-chip:hover {
          opacity: 0.8;
        }

        .suggested-queries ul {
          margin: 0;
          padding-left: 1.2rem;
        }

        .suggested-queries li {
          margin-bottom: 0.25rem;
          color: var(--ifm-color-emphasis-700);
          cursor: pointer;
        }

        .suggested-queries li:hover {
          color: var(--ifm-color-primary);
        }

        .message-meta {
          display: flex;
          align-items: center;
          gap: 0.75rem;
          font-size: 0.75rem;
          color: var(--ifm-color-emphasis-600);
          margin-top: 0.25rem;
        }

        .user-message .message-meta {
          justify-content: flex-end;
        }

        .token-usage {
          cursor: help;
        }

        .filter-message {
          margin-top: 0.5rem;
          font-size: 0.875rem;
          color: var(--ifm-color-emphasis-700);
          display: flex;
          align-items: center;
          gap: 0.5rem;
        }

        .filter-icon {
          font-size: 1em;
        }

        .empty-state {
          display: flex;
          flex-direction: column;
          align-items: center;
          justify-content: center;
          height: 100%;
          text-align: center;
          color: var(--ifm-color-emphasis-600);
        }

        .empty-state-icon {
          font-size: 3rem;
          margin-bottom: 1rem;
          opacity: 0.5;
        }

        .empty-state h3 {
          margin: 0 0 0.5rem 0;
          color: var(--ifm-font-color-base);
        }

        .empty-state p {
          margin: 0;
          max-width: 400px;
        }

        .streaming-indicator {
          display: flex;
          align-items: center;
          gap: 0.25rem;
          padding: 0.5rem 0;
        }

        .dot {
          width: 8px;
          height: 8px;
          border-radius: 50%;
          background-color: var(--ifm-color-emphasis-400);
          animation: pulse 1.4s infinite ease-in-out both;
        }

        .dot:nth-child(1) {
          animation-delay: -0.32s;
        }

        .dot:nth-child(2) {
          animation-delay: -0.16s;
        }

        @keyframes pulse {
          0%,
          80%,
          100% {
            transform: scale(0);
            opacity: 0.5;
          }
          40% {
            transform: scale(1);
            opacity: 1;
          }
        }

        /* Dark mode adjustments */
        [data-theme='dark'] .message-content {
          box-shadow: 0 1px 2px rgba(0, 0, 0, 0.2);
        }

        [data-theme='dark'] .suggested-topics,
        [data-theme='dark'] .suggested-queries {
          background-color: var(--ifm-color-emphasis-200);
        }

        /* Mobile adjustments */
        @media (max-width: 768px) {
          .message-list {
            padding: 0.75rem;
          }

          .message {
            max-width: 95%;
          }

          .message-content {
            padding: 0.625rem 0.875rem;
          }
        }
      `}</style>
    </div>
  );
};
