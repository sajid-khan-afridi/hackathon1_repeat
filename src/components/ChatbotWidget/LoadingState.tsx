/**
 * LoadingState Component
 *
 * Displays loading and streaming indicators for the chatbot widget.
 * Uses the unified TypingIndicator component from SkeletonLoader.
 * Features:
 * - Animated typing indicator for non-streaming loading
 * - Streaming indicator with pulsing dots
 * - WCAG 2.1 AA compliant with aria-live regions
 * - Respects prefers-reduced-motion
 *
 * @see specs/007-enhance-ui/plan.md for architecture
 */

import React from 'react';
import { TypingIndicator } from '../SkeletonLoader';
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
}: LoadingStateProps): React.ReactElement {
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
        {/* Typing indicator - uses TypingIndicator from SkeletonLoader */}
        <TypingIndicator
          size="small"
          label={message || defaultMessage}
        />

        {/* Loading text */}
        <span className={styles.loadingText}>
          {message || defaultMessage}
        </span>
      </div>
    </div>
  );
}
