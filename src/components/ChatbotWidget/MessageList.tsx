/**
 * MessageList Component
 *
 * Scrollable message container with user/assistant bubbles.
 * Features:
 * - Auto-scroll to bottom on new messages
 * - Semantic HTML (<article> for messages)
 * - Loading indicator during query processing
 * - Timestamp display
 * - Integrated source citations and confidence indicators
 * - Keyboard navigation and screen reader support
 */

import React, { useRef, useEffect } from 'react';
import styles from './ChatbotWidget.module.css';
import type { Message } from './types';
import SourceCitations from './SourceCitations';
import ConfidenceIndicator from './ConfidenceIndicator';

interface MessageListProps {
  messages: Message[];
  isLoading: boolean;
}

/**
 * Formats timestamp for display
 */
function formatTimestamp(date: Date): string {
  const now = new Date();
  const diff = now.getTime() - date.getTime();
  const seconds = Math.floor(diff / 1000);
  const minutes = Math.floor(seconds / 60);
  const hours = Math.floor(minutes / 60);

  if (seconds < 60) return 'Just now';
  if (minutes < 60) return `${minutes}m ago`;
  if (hours < 24) return `${hours}h ago`;

  // Show full time if older than 24 hours
  return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
}

export default function MessageList({ messages, isLoading }: MessageListProps): React.ReactElement {
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);

  /**
   * Auto-scroll to bottom when new messages arrive
   * Uses smooth scrolling for better UX
   */
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  /**
   * Manage focus for screen readers when new assistant messages arrive
   */
  useEffect(() => {
    if (messages.length > 0) {
      const lastMessage = messages[messages.length - 1];
      if (lastMessage.role === 'assistant') {
        // Announce new message to screen readers
        const announcement = document.getElementById('message-announcement');
        if (announcement) {
          announcement.textContent = `New response received`;
        }
      }
    }
  }, [messages]);

  return (
    <div
      ref={containerRef}
      className={styles.messageList}
      role="log"
      aria-label="Chat messages"
      aria-live="polite"
      aria-atomic="false"
    >
      {/* Screen reader announcement area */}
      <div
        id="message-announcement"
        className={styles.visuallyHidden}
        aria-live="assertive"
        aria-atomic="true"
      />

      {/* Empty state */}
      {messages.length === 0 && !isLoading && (
        <div className={styles.emptyState} role="status">
          <svg
            width="48"
            height="48"
            viewBox="0 0 48 48"
            fill="currentColor"
            aria-hidden="true"
            className={styles.emptyStateIcon}
          >
            <path d="M24 4C13 4 4 13 4 24s9 20 20 20 20-9 20-20S35 4 24 4zm0 36c-8.8 0-16-7.2-16-16S15.2 8 24 8s16 7.2 16 16-7.2 16-16 16zm-2-26h4v12h-4zm0 16h4v4h-4z" />
          </svg>
          <p>Ask a question about the robotics textbook to get started.</p>
        </div>
      )}

      {/* Message bubbles */}
      {messages.map((message) => (
        <article
          key={message.id}
          className={`${styles.message} ${styles[`message-${message.role}`]}`}
          aria-label={`${message.role === 'user' ? 'Your question' : 'Assistant response'}`}
        >
          <div className={styles.messageHeader}>
            <span className={styles.messageRole}>
              {message.role === 'user' ? 'You' : 'Assistant'}
            </span>
            <time className={styles.messageTimestamp} dateTime={message.timestamp.toISOString()}>
              {formatTimestamp(message.timestamp)}
            </time>
          </div>

          <div className={styles.messageContent}>{message.content}</div>

          {/* Assistant-only metadata */}
          {message.role === 'assistant' && (
            <>
              {/* Confidence indicator */}
              {message.confidence !== undefined && (
                <ConfidenceIndicator
                  confidence={message.confidence}
                  showWarning={message.confidence >= 0.2 && message.confidence < 0.3}
                />
              )}

              {/* Source citations */}
              {message.sources && message.sources.length > 0 && (
                <SourceCitations sources={message.sources} />
              )}

              {/* Token usage (educational transparency) */}
              {message.tokens_used && (
                <div className={styles.tokenUsage} aria-label="Token usage information">
                  <span className={styles.tokenUsageLabel}>Tokens:</span>
                  <span className={styles.tokenUsageValue}>
                    {message.tokens_used.total_tokens}
                    <span className={styles.tokenUsageBreakdown}>
                      ({message.tokens_used.input_tokens} in / {message.tokens_used.output_tokens}{' '}
                      out)
                    </span>
                  </span>
                </div>
              )}

              {/* Adaptive filtering message */}
              {message.filter_message && (
                <div className={styles.filterMessage} role="status">
                  <svg
                    width="16"
                    height="16"
                    viewBox="0 0 16 16"
                    fill="currentColor"
                    aria-hidden="true"
                  >
                    <path d="M8 0C3.6 0 0 3.6 0 8s3.6 8 8 8 8-3.6 8-8-3.6-8-8-8zm1 12H7v-2h2v2zm0-4H7V4h2v4z" />
                  </svg>
                  {message.filter_message}
                </div>
              )}

              {/* Suggested terms for off-topic queries */}
              {message.suggested_terms && message.suggested_terms.length > 0 && (
                <div className={styles.suggestedTerms}>
                  <p className={styles.suggestedTermsLabel}>Try asking about:</p>
                  <ul className={styles.suggestedTermsList}>
                    {message.suggested_terms.map((term, idx) => (
                      <li key={idx}>
                        <button
                          className={styles.suggestedTermButton}
                          onClick={() => {
                            // Dispatch custom event for parent to handle
                            const event = new CustomEvent('chatbot-suggested-term', {
                              detail: { term },
                            });
                            window.dispatchEvent(event);
                          }}
                          type="button"
                        >
                          {term}
                        </button>
                      </li>
                    ))}
                  </ul>
                </div>
              )}
            </>
          )}
        </article>
      ))}

      {/* Loading indicator */}
      {isLoading && (
        <div
          className={styles.loadingIndicator}
          role="status"
          aria-live="polite"
          aria-label="Loading response"
        >
          <div className={styles.loadingDots}>
            <span></span>
            <span></span>
            <span></span>
          </div>
          <span className={styles.visuallyHidden}>Loading response...</span>
        </div>
      )}

      {/* Scroll anchor */}
      <div ref={messagesEndRef} />
    </div>
  );
}
