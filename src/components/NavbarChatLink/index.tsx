/**
 * NavbarChatLink Component
 *
 * Alternative access to AI Assistant chatbot via navbar link.
 * Features:
 * - Desktop: "AI Assistant" link in navbar
 * - Mobile: Icon button in mobile menu
 * - Opens the floating chat widget when clicked
 * - Keyboard accessible
 * - Screen reader support
 * - Docusaurus theme integration
 */

import React from 'react';
import styles from './styles.module.css';

interface NavbarChatLinkProps {
  /** Display mode (link or icon) */
  mode?: 'link' | 'icon';
  /** Custom label text */
  label?: string;
  /** Click handler to open chat */
  onClick?: () => void;
}

export default function NavbarChatLink({
  mode = 'link',
  label = 'AI Assistant',
  onClick
}: NavbarChatLinkProps): JSX.Element {
  const handleClick = (e: React.MouseEvent) => {
    e.preventDefault();

    if (onClick) {
      onClick();
    } else {
      // Dispatch custom event to open floating chat
      window.dispatchEvent(new CustomEvent('open-floating-chat'));
    }
  };

  return (
    <a
      href="#ai-assistant"
      onClick={handleClick}
      className={`${styles.navbarChatLink} ${mode === 'icon' ? styles.navbarChatLinkIcon : ''}`}
      aria-label={label}
      role="button"
      tabIndex={0}
      onKeyDown={(e) => {
        if (e.key === 'Enter' || e.key === ' ') {
          e.preventDefault();
          handleClick(e as any);
        }
      }}
    >
      {/* Robot/Chat Icon */}
      <svg
        width="20"
        height="20"
        viewBox="0 0 24 24"
        fill="currentColor"
        className={styles.icon}
        aria-hidden="true"
      >
        <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm0 3c1.66 0 3 1.34 3 3s-1.34 3-3 3-3-1.34-3-3 1.34-3 3-3zm0 14.2c-2.5 0-4.71-1.28-6-3.22.03-1.99 4-3.08 6-3.08 1.99 0 5.97 1.09 6 3.08-1.29 1.94-3.5 3.22-6 3.22z"/>
      </svg>

      {/* Label (shown in link mode, hidden in icon mode) */}
      {mode === 'link' && <span className={styles.label}>{label}</span>}

      {/* Screen reader text for icon mode */}
      {mode === 'icon' && <span className={styles.visuallyHidden}>{label}</span>}
    </a>
  );
}
