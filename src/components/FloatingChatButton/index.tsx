/**
 * FloatingChatButton Component
 *
 * Fixed-position floating action button (FAB) to open AI Assistant chat.
 * Features:
 * - Fixed bottom-right position (responsive spacing)
 * - Robot/chat icon with optional label
 * - Ripple effect on click
 * - Hover animations (scale, shadow)
 * - WCAG 2.1 AA compliant (44x44px touch target)
 * - Reduced motion support
 */

import React from 'react';
import styles from './styles.module.css';

interface FloatingChatButtonProps {
  /** Click handler to open chat popup */
  onClick: () => void;
  /** Whether button is currently expanded (for ARIA state) */
  isExpanded: boolean;
  /** Unread message count (optional, shows badge if > 0) */
  unreadCount?: number;
}

export default function FloatingChatButton({
  onClick,
  isExpanded,
  unreadCount = 0
}: FloatingChatButtonProps): JSX.Element {
  const hasUnread = unreadCount > 0;
  const ariaLabel = hasUnread
    ? `Open AI Assistant chat, ${unreadCount} unread message${unreadCount > 1 ? 's' : ''}`
    : 'Open AI Assistant chat';

  return (
    <button
      onClick={onClick}
      className={styles.floatingButton}
      aria-label={ariaLabel}
      aria-expanded={isExpanded}
      aria-haspopup="dialog"
      type="button"
    >
      {/* Robot/Chat Icon (IoMdChatbubbles inspired) */}
      <svg
        width="28"
        height="28"
        viewBox="0 0 512 512"
        fill="currentColor"
        aria-hidden="true"
        className={styles.buttonIcon}
      >
        <path d="M144 208c-17.7 0-32 14.3-32 32s14.3 32 32 32 32-14.3 32-32-14.3-32-32-32zm224 0c-17.7 0-32 14.3-32 32s14.3 32 32 32 32-14.3 32-32-14.3-32-32-32zm112-48v192c0 35.3-28.7 64-64 64h-86.7l-97.3 73-97.3-73H48c-35.3 0-64-28.7-64-64V160c0-35.3 28.7-64 64-64h368c35.3 0 64 28.7 64 64zm-32 0c0-17.6-14.4-32-32-32H48c-17.6 0-32 14.4-32 32v192c0 17.6 14.4 32 32 32h106.7L256 448l101.3-64H416c17.6 0 32-14.4 32-32V160z"/>
      </svg>

      {/* Label (visible on desktop) */}
      <span className={styles.buttonLabel}>AI Assistant</span>

      {/* Unread Badge */}
      {hasUnread && (
        <span className={styles.badge} aria-hidden="true">
          {unreadCount > 9 ? '9+' : unreadCount}
        </span>
      )}

      {/* Ripple effect container */}
      <span className={styles.ripple} aria-hidden="true"></span>
    </button>
  );
}
