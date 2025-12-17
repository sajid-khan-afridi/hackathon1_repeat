/**
 * LoadingState Component
 *
 * Displays loading and streaming indicators for the chatbot widget.
 * Features:
 * - Animated typing indicator for non-streaming loading
 * - Streaming indicator with pulsing dots
 * - WCAG 2.1 AA compliant with aria-live regions
 */

import React from 'react';
import styles from './ChatbotWidget.module.css';

interface LoadingStateProps {
  /** Whether the response is streaming (vs initial loading) */
  isStreaming?: boolean;
  /** Optional custom loading message */
  message?: string;
}

export default function LoadingState({
  isStreaming = false,
  message
}: LoadingStateProps): JSX.Element {
  const defaultMessage = isStreaming
    ? 'Streaming response...'
    : 'Thinking...';

  return (
    <div
      className={styles.loadingState}
      role="status"
      aria-live="polite"
      aria-label={message || defaultMessage}
    >
      <div className={styles.loadingContent}>
        {/* Animated dots indicator */}
        <div className={styles.loadingDots}>
          <span className={styles.dot} aria-hidden="true" />
          <span className={styles.dot} aria-hidden="true" />
          <span className={styles.dot} aria-hidden="true" />
        </div>

        {/* Loading text */}
        <span className={styles.loadingText}>
          {message || defaultMessage}
        </span>
      </div>

      {/* Screen reader only text */}
      <span className={styles.srOnly}>
        {isStreaming
          ? 'Response is streaming, please wait'
          : 'Generating response, please wait'}
      </span>
    </div>
  );
}
