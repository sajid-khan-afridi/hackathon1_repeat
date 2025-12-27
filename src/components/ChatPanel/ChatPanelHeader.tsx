/**
 * ChatPanelHeader Component
 *
 * Header for the docked chat panel with controls for minimize, expand, and close.
 *
 * @see specs/007-enhance-ui/data-model.md for component props
 */

import React from 'react';
import { Minus, Maximize2, X, Search } from 'lucide-react';
import clsx from 'clsx';
import styles from './ChatPanelHeader.module.css';

export interface ChatPanelHeaderProps {
  /** Panel title */
  title?: string;

  /** Whether panel is minimized */
  isMinimized: boolean;

  /** Whether search is active */
  isSearchActive?: boolean;

  /** Callback when minimize button clicked */
  onMinimize: () => void;

  /** Callback when expand button clicked */
  onExpand: () => void;

  /** Callback when close button clicked */
  onClose: () => void;

  /** Callback when search button clicked */
  onSearchToggle?: () => void;

  /** Additional CSS classes */
  className?: string;
}

/**
 * ChatPanelHeader Component
 *
 * Provides header controls for the chat panel.
 */
export function ChatPanelHeader({
  title = 'AI Assistant',
  isMinimized,
  isSearchActive = false,
  onMinimize,
  onExpand,
  onClose,
  onSearchToggle,
  className,
}: ChatPanelHeaderProps): React.ReactElement {
  return (
    <header className={clsx(styles.header, { [styles.minimized]: isMinimized }, className)}>
      <div className={styles.titleSection}>
        <span className={styles.title}>{title}</span>
      </div>

      <div className={styles.controls}>
        {!isMinimized && onSearchToggle && (
          <button
            type="button"
            className={clsx(styles.controlButton, { [styles.active]: isSearchActive })}
            onClick={onSearchToggle}
            aria-label="Search chat history"
            aria-pressed={isSearchActive}
            title="Search (Ctrl+F)"
          >
            <Search size={18} aria-hidden="true" />
          </button>
        )}

        {isMinimized ? (
          <button
            type="button"
            className={styles.controlButton}
            onClick={onExpand}
            aria-label="Expand chat panel"
            title="Expand"
          >
            <Maximize2 size={18} aria-hidden="true" />
          </button>
        ) : (
          <button
            type="button"
            className={styles.controlButton}
            onClick={onMinimize}
            aria-label="Minimize chat panel"
            title="Minimize"
          >
            <Minus size={18} aria-hidden="true" />
          </button>
        )}

        <button
          type="button"
          className={clsx(styles.controlButton, styles.closeButton)}
          onClick={onClose}
          aria-label="Close chat panel"
          title="Close"
        >
          <X size={18} aria-hidden="true" />
        </button>
      </div>
    </header>
  );
}

export default ChatPanelHeader;
