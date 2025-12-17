/**
 * Enhanced FloatingChatPopup Component
 *
 * Complete floating chatbot widget with all 10 requirements:
 * 1. FAB with hover/ripple effects and unread badge
 * 2. Chat window with smooth animations (380-420px, max 70vh)
 * 3. Branded gradient header with "Robotics AI Assistant" title
 * 4. Bot identity message (first message)
 * 5. Quick action chips for guided flows
 * 6. Message UI with avatars, bubbles, timestamps, code blocks
 * 7. Input field with send button, disabled/loading states
 * 8. Responsive design (mobile: full-screen, desktop: corner widget, drag-to-move)
 * 9. Full accessibility (ARIA, focus trap, ESC to close)
 * 10. Alternative access via navbar
 */

import React, { useEffect, useRef, useState, useCallback } from 'react';
import ChatbotWidget from '../ChatbotWidget';
import QuickActionChips from '../ChatbotWidget/QuickActionChips';
import styles from './styles.module.css';

interface EnhancedFloatingChatPopupProps {
  /** Whether popup is visible */
  isOpen: boolean;
  /** Close handler */
  onClose: () => void;
  /** Animation state (opening, open, closing, closed) */
  animationState: 'opening' | 'open' | 'closing' | 'closed';
}

export default function EnhancedFloatingChatPopup({
  isOpen,
  onClose,
  animationState
}: EnhancedFloatingChatPopupProps): JSX.Element | null {
  const popupRef = useRef<HTMLDivElement>(null);
  const headerRef = useRef<HTMLDivElement>(null);
  const closeButtonRef = useRef<HTMLButtonElement>(null);
  const previousFocusRef = useRef<HTMLElement | null>(null);

  // State for drag-to-move
  const [position, setPosition] = useState<{ x: number; y: number } | null>(null);
  const [isDragging, setIsDragging] = useState(false);
  const [dragOffset, setDragOffset] = useState({ x: 0, y: 0 });
  const [isMinimized, setIsMinimized] = useState(false);
  const [showMenu, setShowMenu] = useState(false);

  /**
   * Drag-to-move handlers (desktop only)
   */
  const handleMouseDown = useCallback((e: React.MouseEvent) => {
    // Only enable drag on desktop and when clicking header
    if (window.innerWidth < 1024) return;
    if (!headerRef.current?.contains(e.target as Node)) return;
    if ((e.target as HTMLElement).closest('button')) return; // Don't drag when clicking buttons

    setIsDragging(true);
    const rect = popupRef.current?.getBoundingClientRect();
    if (rect) {
      setDragOffset({
        x: e.clientX - rect.left,
        y: e.clientY - rect.top
      });
    }
  }, []);

  useEffect(() => {
    if (!isDragging) return;

    const handleMouseMove = (e: MouseEvent) => {
      setPosition({
        x: e.clientX - dragOffset.x,
        y: e.clientY - dragOffset.y
      });
    };

    const handleMouseUp = () => {
      setIsDragging(false);
    };

    document.addEventListener('mousemove', handleMouseMove);
    document.addEventListener('mouseup', handleMouseUp);

    return () => {
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, [isDragging, dragOffset]);

  /**
   * Focus management: trap focus inside popup when open
   */
  useEffect(() => {
    if (isOpen && animationState === 'open') {
      // Store previously focused element
      previousFocusRef.current = document.activeElement as HTMLElement;

      // Focus close button after animation completes
      setTimeout(() => {
        closeButtonRef.current?.focus();
      }, 350);

      // Focus trap handler
      const handleFocusTrap = (e: KeyboardEvent) => {
        if (e.key !== 'Tab') return;

        const focusableElements = popupRef.current?.querySelectorAll(
          'button:not([disabled]), [href], input:not([disabled]), textarea:not([disabled]), [tabindex]:not([tabindex="-1"])'
        );

        if (!focusableElements || focusableElements.length === 0) return;

        const firstElement = focusableElements[0] as HTMLElement;
        const lastElement = focusableElements[focusableElements.length - 1] as HTMLElement;

        if (e.shiftKey && document.activeElement === firstElement) {
          e.preventDefault();
          lastElement.focus();
        } else if (!e.shiftKey && document.activeElement === lastElement) {
          e.preventDefault();
          firstElement.focus();
        }
      };

      document.addEventListener('keydown', handleFocusTrap);

      return () => {
        document.removeEventListener('keydown', handleFocusTrap);
      };
    } else if (!isOpen && previousFocusRef.current) {
      // Restore focus when popup closes
      previousFocusRef.current.focus();
      previousFocusRef.current = null;
    }
  }, [isOpen, animationState]);

  /**
   * ESC key handler
   */
  useEffect(() => {
    if (!isOpen) return;

    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        e.preventDefault();
        e.stopPropagation();

        // Close menu if open, otherwise close popup
        if (showMenu) {
          setShowMenu(false);
        } else {
          onClose();
        }
      }
    };

    document.addEventListener('keydown', handleEscape);
    return () => {
      document.removeEventListener('keydown', handleEscape);
    };
  }, [isOpen, showMenu, onClose]);

  /**
   * Prevent body scroll when popup is open (mobile only)
   */
  useEffect(() => {
    if (isOpen && window.innerWidth < 768) {
      document.body.style.overflow = 'hidden';
      document.body.style.position = 'fixed';
      document.body.style.width = '100%';
    } else {
      document.body.style.overflow = '';
      document.body.style.position = '';
      document.body.style.width = '';
    }

    return () => {
      document.body.style.overflow = '';
      document.body.style.position = '';
      document.body.style.width = '';
    };
  }, [isOpen]);

  /**
   * Handle minimize toggle
   */
  const handleMinimize = useCallback(() => {
    setIsMinimized(prev => !prev);
    // Announce to screen readers
    const message = isMinimized ? 'Chat expanded' : 'Chat minimized';
    announceToScreenReader(message);
  }, [isMinimized]);

  /**
   * Handle menu toggle
   */
  const handleMenuToggle = useCallback(() => {
    setShowMenu(prev => !prev);
  }, []);

  // Don't render if closed
  if (!isOpen && animationState === 'closed') {
    return null;
  }

  // Calculate position styles for drag
  const positionStyles = position && window.innerWidth >= 1024
    ? {
        right: 'auto',
        bottom: 'auto',
        left: `${position.x}px`,
        top: `${position.y}px`
      }
    : {};

  return (
    <>
      {/* Backdrop (mobile only) */}
      <div
        className={`${styles.backdrop} ${styles[`backdrop-${animationState}`]}`}
        onClick={onClose}
        aria-hidden="true"
      />

      {/* Popup container */}
      <div
        ref={popupRef}
        className={`${styles.popup} ${styles[`popup-${animationState}`]} ${isMinimized ? styles.popupMinimized : ''} ${isDragging ? styles.popupDragging : ''}`}
        style={positionStyles}
        role="dialog"
        aria-modal="true"
        aria-labelledby="floating-chat-title"
        aria-describedby="floating-chat-description"
        onMouseDown={handleMouseDown}
      >
        {/* Branded Header */}
        <div ref={headerRef} className={styles.headerBranded}>
          <div className={styles.headerContent}>
            {/* Bot Avatar */}
            <div className={styles.botAvatar} aria-hidden="true">
              <svg width="24" height="24" viewBox="0 0 24 24" fill="white">
                <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm0 3c1.66 0 3 1.34 3 3s-1.34 3-3 3-3-1.34-3-3 1.34-3 3-3zm0 14.2c-2.5 0-4.71-1.28-6-3.22.03-1.99 4-3.08 6-3.08 1.99 0 5.97 1.09 6 3.08-1.29 1.94-3.5 3.22-6 3.22z"/>
              </svg>
            </div>

            {/* Title */}
            <h2 id="floating-chat-title" className={styles.titleBranded}>
              Robotics AI Assistant
            </h2>

            {/* Controls */}
            <div className={styles.headerButtons}>
              {/* Menu button */}
              <button
                onClick={handleMenuToggle}
                className={styles.iconButton}
                aria-label="Menu"
                aria-expanded={showMenu}
                aria-haspopup="true"
                type="button"
              >
                <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor" aria-hidden="true">
                  <path d="M10 6a2 2 0 110-4 2 2 0 010 4zm0 6a2 2 0 110-4 2 2 0 010 4zm0 6a2 2 0 110-4 2 2 0 010 4z"/>
                </svg>
              </button>

              {/* Minimize button (desktop only) */}
              <button
                onClick={handleMinimize}
                className={`${styles.iconButton} ${styles.iconButtonDesktopOnly}`}
                aria-label={isMinimized ? 'Maximize' : 'Minimize'}
                type="button"
              >
                <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor" aria-hidden="true">
                  {isMinimized ? (
                    <path d="M3 3h14v2H3V3zm0 6h14v2H3V9zm0 6h14v2H3v-2z"/>
                  ) : (
                    <path d="M3 10h14v2H3v-2z"/>
                  )}
                </svg>
              </button>

              {/* Close button */}
              <button
                ref={closeButtonRef}
                onClick={onClose}
                className={styles.iconButton}
                aria-label="Close AI Assistant"
                type="button"
              >
                <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor" aria-hidden="true">
                  <path d="M14.348 5.652a.5.5 0 0 0-.707 0L10 9.293 6.354 5.652a.5.5 0 1 0-.707.707L9.293 10l-3.646 3.646a.5.5 0 0 0 .707.707L10 10.707l3.646 3.646a.5.5 0 0 0 .707-.707L10.707 10l3.646-3.646a.5.5 0 0 0 0-.707z"/>
                </svg>
              </button>
            </div>
          </div>

          {/* Drag indicator (desktop only) */}
          <div className={styles.dragIndicator} aria-hidden="true">
            <svg width="16" height="4" viewBox="0 0 16 4" fill="currentColor" opacity="0.3">
              <circle cx="2" cy="2" r="1.5"/>
              <circle cx="8" cy="2" r="1.5"/>
              <circle cx="14" cy="2" r="1.5"/>
            </svg>
          </div>
        </div>

        {/* Menu dropdown */}
        {showMenu && (
          <div className={styles.menuDropdown} role="menu">
            <button
              role="menuitem"
              className={styles.menuItem}
              onClick={() => {
                setShowMenu(false);
                // Trigger clear history
                window.dispatchEvent(new CustomEvent('chatbot-clear-history'));
              }}
            >
              <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
                <path d="M2 3h12v2H2V3zm0 4h12v2H2V7zm0 4h12v2H2v-2z"/>
              </svg>
              Clear History
            </button>
            <button
              role="menuitem"
              className={styles.menuItem}
              onClick={() => {
                setShowMenu(false);
                window.open('/docs/help', '_blank');
              }}
            >
              <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
                <path d="M8 0a8 8 0 100 16A8 8 0 008 0zm1 12H7v-2h2v2zm0-3H7V4h2v5z"/>
              </svg>
              Help & FAQ
            </button>
          </div>
        )}

        {/* Hidden description for screen readers */}
        <p id="floating-chat-description" className={styles.visuallyHidden}>
          Ask questions about the robotics textbook and receive AI-powered answers with source citations. Use arrow keys and Tab to navigate, Escape to close.
        </p>

        {/* Chat Container */}
        {!isMinimized && (
          <div className={styles.chatContainer}>
            {/* Bot Identity Message (shown on first load) */}
            <div className={styles.botIdentity}>
              <div className={styles.botIdentityIcon} aria-hidden="true">
                🤖
              </div>
              <div className={styles.botIdentityText}>
                <strong>Hi! I'm your Robotics AI Assistant.</strong>
                <p>
                  I can help you understand robotics concepts, find relevant modules,
                  explain ROS 2, and answer questions from the textbook. I'll provide
                  answers with source citations when available.
                </p>
                <p className={styles.botIdentityNote}>
                  <strong>What I can't do:</strong> I can't perform calculations,
                  access external resources, or provide real-time data.
                </p>
              </div>
            </div>

            {/* Quick Action Chips */}
            <QuickActionChips
              onActionClick={(query) => {
                // Dispatch event to ChatbotWidget to handle query
                window.dispatchEvent(new CustomEvent('chatbot-suggested-term', {
                  detail: { term: query }
                }));
              }}
            />

            {/* ChatbotWidget */}
            <ChatbotWidget />
          </div>
        )}
      </div>
    </>
  );
}

/**
 * Helper: Announce message to screen readers
 */
function announceToScreenReader(message: string): void {
  const announcer = document.getElementById('floating-chat-announcer');
  if (announcer) {
    announcer.textContent = message;
    setTimeout(() => {
      announcer.textContent = '';
    }, 1000);
  }
}
