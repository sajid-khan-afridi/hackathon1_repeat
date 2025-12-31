/**
 * FloatingChatPopup Component
 *
 * Modal popup containing the ChatbotWidget with animations and header controls.
 * Features:
 * - Animated entrance/exit (fade, scale, slide)
 * - Responsive sizing (desktop: 380x600px, mobile: bottom sheet)
 * - Header with close and clear history buttons
 * - Reuses existing ChatbotWidget component
 * - Focus trap for accessibility
 * - ESC key to close
 * - Dark mode support
 */

import React, { useEffect, useRef } from 'react';
import ChatbotWidget from '../ChatbotWidget';
import styles from './styles.module.css';

interface FloatingChatPopupProps {
  /** Whether popup is visible */
  isOpen: boolean;
  /** Close handler */
  onClose: () => void;
  /** Animation state (opening, open, closing, closed) */
  animationState: 'opening' | 'open' | 'closing' | 'closed';
}

export default function FloatingChatPopup({
  isOpen,
  onClose,
  animationState,
}: FloatingChatPopupProps): JSX.Element | null {
  const popupRef = useRef<HTMLDivElement>(null);
  const closeButtonRef = useRef<HTMLButtonElement>(null);
  const previousFocusRef = useRef<HTMLElement | null>(null);

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
      }, 350); // After 300ms animation + small delay

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
        onClose();
      }
    };

    document.addEventListener('keydown', handleEscape);
    return () => {
      document.removeEventListener('keydown', handleEscape);
    };
  }, [isOpen, onClose]);

  /**
   * Prevent body scroll when popup is open
   */
  useEffect(() => {
    if (isOpen) {
      document.body.style.overflow = 'hidden';
      // Add iOS safari specific fix
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

  // Don't render if closed
  if (!isOpen && animationState === 'closed') {
    return null;
  }

  return (
    <>
      {/* Backdrop */}
      <div
        className={`${styles.backdrop} ${styles[`backdrop-${animationState}`]}`}
        onClick={onClose}
        aria-hidden="true"
      />

      {/* Popup container */}
      <div
        ref={popupRef}
        className={`${styles.popup} ${styles[`popup-${animationState}`]}`}
        role="dialog"
        aria-modal="true"
        aria-labelledby="floating-chat-title"
        aria-describedby="floating-chat-description"
      >
        {/* Header */}
        <div className={styles.header}>
          <h2 id="floating-chat-title" className={styles.title}>
            AI Assistant
          </h2>

          <div className={styles.headerButtons}>
            {/* Close button */}
            <button
              ref={closeButtonRef}
              onClick={onClose}
              className={styles.iconButton}
              aria-label="Close AI Assistant"
              type="button"
            >
              <svg
                width="20"
                height="20"
                viewBox="0 0 20 20"
                fill="currentColor"
                aria-hidden="true"
              >
                <path d="M14.348 5.652a.5.5 0 0 0-.707 0L10 9.293 6.354 5.652a.5.5 0 1 0-.707.707L9.293 10l-3.646 3.646a.5.5 0 0 0 .707.707L10 10.707l3.646 3.646a.5.5 0 0 0 .707-.707L10.707 10l3.646-3.646a.5.5 0 0 0 0-.707z" />
              </svg>
            </button>
          </div>
        </div>

        {/* Hidden description for screen readers */}
        <p id="floating-chat-description" className={styles.visuallyHidden}>
          Ask questions about the robotics textbook and receive AI-powered answers with source
          citations.
        </p>

        {/* ChatbotWidget (popup mode) */}
        <div className={styles.chatContainer}>
          <ChatbotWidget />
        </div>
      </div>
    </>
  );
}
