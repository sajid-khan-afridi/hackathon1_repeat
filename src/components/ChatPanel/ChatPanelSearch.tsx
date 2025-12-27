/**
 * ChatPanelSearch Component
 *
 * Search input and navigation for chat history.
 *
 * @see specs/007-enhance-ui/data-model.md for search state
 */

import React, { useRef, useEffect, useCallback } from 'react';
import { Search, ChevronUp, ChevronDown, X } from 'lucide-react';
import clsx from 'clsx';
import styles from './ChatPanelSearch.module.css';

export interface ChatPanelSearchProps {
  /** Current search query */
  query: string;

  /** Callback when query changes */
  onQueryChange: (query: string) => void;

  /** Current result index (0-based) */
  activeIndex: number;

  /** Total number of results */
  resultCount: number;

  /** Callback to go to next result */
  onNextResult: () => void;

  /** Callback to go to previous result */
  onPrevResult: () => void;

  /** Callback to close search */
  onClose: () => void;

  /** Whether search panel is visible */
  isVisible: boolean;

  /** Additional CSS classes */
  className?: string;
}

/**
 * ChatPanelSearch Component
 *
 * Provides search functionality within chat history.
 * Supports keyboard shortcuts (Ctrl+F to open, Escape to close).
 */
export function ChatPanelSearch({
  query,
  onQueryChange,
  activeIndex,
  resultCount,
  onNextResult,
  onPrevResult,
  onClose,
  isVisible,
  className,
}: ChatPanelSearchProps): React.ReactElement | null {
  const inputRef = useRef<HTMLInputElement>(null);

  // Focus input when search becomes visible
  useEffect(() => {
    if (isVisible && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isVisible]);

  // Handle keyboard shortcuts
  const handleKeyDown = useCallback(
    (e: React.KeyboardEvent) => {
      switch (e.key) {
        case 'Escape':
          onClose();
          break;
        case 'Enter':
          if (e.shiftKey) {
            onPrevResult();
          } else {
            onNextResult();
          }
          break;
        case 'F3':
          e.preventDefault();
          if (e.shiftKey) {
            onPrevResult();
          } else {
            onNextResult();
          }
          break;
        default:
          break;
      }
    },
    [onClose, onNextResult, onPrevResult]
  );

  if (!isVisible) {
    return null;
  }

  return (
    <div className={clsx(styles.container, className)} role="search">
      <div className={styles.inputWrapper}>
        <Search size={16} className={styles.icon} aria-hidden="true" />
        <input
          ref={inputRef}
          type="text"
          className={styles.input}
          placeholder="Search messages..."
          value={query}
          onChange={(e) => onQueryChange(e.target.value)}
          onKeyDown={handleKeyDown}
          aria-label="Search chat history"
          aria-describedby="search-results-count"
        />
      </div>

      <div className={styles.results} id="search-results-count">
        {query.length >= 2 && (
          <span className={styles.resultCount}>
            {resultCount > 0
              ? `${activeIndex + 1}/${resultCount}`
              : 'No results'}
          </span>
        )}
      </div>

      <div className={styles.navigation}>
        <button
          type="button"
          className={styles.navButton}
          onClick={onPrevResult}
          disabled={resultCount === 0}
          aria-label="Previous result"
          title="Previous (Shift+Enter)"
        >
          <ChevronUp size={18} aria-hidden="true" />
        </button>
        <button
          type="button"
          className={styles.navButton}
          onClick={onNextResult}
          disabled={resultCount === 0}
          aria-label="Next result"
          title="Next (Enter)"
        >
          <ChevronDown size={18} aria-hidden="true" />
        </button>
        <button
          type="button"
          className={clsx(styles.navButton, styles.closeButton)}
          onClick={onClose}
          aria-label="Close search"
          title="Close (Escape)"
        >
          <X size={18} aria-hidden="true" />
        </button>
      </div>
    </div>
  );
}

export default ChatPanelSearch;
