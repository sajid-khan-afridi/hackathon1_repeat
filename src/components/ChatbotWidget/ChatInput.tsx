/**
 * ChatInput Component
 *
 * Accessible textarea input with submit button for chat queries.
 * Features:
 * - Auto-resize textarea (min 2 lines, max 6 lines)
 * - Enter to submit, Shift+Enter for newline
 * - Character counter (max 1000 characters)
 * - Disabled state during loading
 * - ARIA labels for screen readers
 * - 44x44px touch targets for mobile accessibility
 */

import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatbotWidget.module.css';

interface ChatInputProps {
  onSubmit: (query: string) => void;
  isLoading: boolean;
  /** For accessibility: associates input with label */
  inputId?: string;
}

const MAX_CHARS = 1000;
const MIN_ROWS = 2;
const MAX_ROWS = 6;

export default function ChatInput({
  onSubmit,
  isLoading,
  inputId = 'chatbot-input'
}: ChatInputProps): React.ReactElement {
  const [value, setValue] = useState('');
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  /**
   * Auto-resize textarea based on content
   * Maintains min/max row constraints for UX
   */
  useEffect(() => {
    const textarea = textareaRef.current;
    if (!textarea) return;

    // Reset height to auto to get correct scrollHeight
    textarea.style.height = 'auto';

    // Calculate new height based on content
    const lineHeight = parseInt(getComputedStyle(textarea).lineHeight);
    const minHeight = lineHeight * MIN_ROWS;
    const maxHeight = lineHeight * MAX_ROWS;
    const newHeight = Math.min(Math.max(textarea.scrollHeight, minHeight), maxHeight);

    textarea.style.height = `${newHeight}px`;
  }, [value]);

  const handleChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    const newValue = e.target.value;
    // Enforce character limit
    if (newValue.length <= MAX_CHARS) {
      setValue(newValue);
    }
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    const trimmedValue = value.trim();

    if (trimmedValue && !isLoading) {
      onSubmit(trimmedValue);
      setValue(''); // Clear input after submit
    }
  };

  /**
   * Keyboard navigation:
   * - Enter: Submit (unless Shift is held)
   * - Shift+Enter: Insert newline
   */
  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e as any);
    }
  };

  const remainingChars = MAX_CHARS - value.length;
  const isNearLimit = remainingChars < 100;
  const canSubmit = value.trim().length > 0 && !isLoading;

  return (
    <form
      onSubmit={handleSubmit}
      className={styles.chatInputForm}
      role="search"
      aria-label="Ask a question about the textbook"
    >
      <div className={styles.inputWrapper}>
        <label htmlFor={inputId} className={styles.visuallyHidden}>
          Ask a question (max {MAX_CHARS} characters)
        </label>

        <textarea
          ref={textareaRef}
          id={inputId}
          value={value}
          onChange={handleChange}
          onKeyDown={handleKeyDown}
          placeholder="Ask a question about robotics..."
          disabled={isLoading}
          className={styles.chatInput}
          rows={MIN_ROWS}
          aria-describedby={`${inputId}-counter`}
          aria-invalid={false}
          // Accessibility: Prevent textarea from being unintentionally submitted
          aria-multiline="true"
        />

        {/* Character counter - visible when approaching limit */}
        <div
          id={`${inputId}-counter`}
          className={`${styles.charCounter} ${isNearLimit ? styles.charCounterWarning : ''}`}
          aria-live="polite"
          aria-atomic="true"
        >
          {isNearLimit && (
            <span>
              {remainingChars} characters remaining
            </span>
          )}
        </div>
      </div>

      <button
        type="submit"
        disabled={!canSubmit}
        className={styles.submitButton}
        aria-label={isLoading ? 'Sending...' : 'Send message'}
        // Accessibility: 44x44px minimum touch target enforced via CSS
      >
        {isLoading ? (
          <span className={styles.loadingDots} aria-hidden="true">
            ···
          </span>
        ) : (
          <svg
            width="20"
            height="20"
            viewBox="0 0 20 20"
            fill="currentColor"
            aria-hidden="true"
          >
            <path d="M2 3l16 7-16 7V3zm2 11.5V5.5l10.5 4.5L4 14.5z" />
          </svg>
        )}
        <span className={styles.visuallyHidden}>
          {isLoading ? 'Sending message' : 'Send message'}
        </span>
      </button>
    </form>
  );
}
