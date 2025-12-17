/**
 * GlobalFloatingChat Component
 *
 * Global wrapper managing floating chat button and popup.
 * Features:
 * - State management (open/closed)
 * - Animation state machine (opening, open, closing, closed)
 * - localStorage persistence for open/closed preference
 * - Announcement for screen readers
 * - Click outside to close (optional)
 * - ESC key handling
 * - SSR compatibility
 */

import React, { useState, useEffect, useCallback } from 'react';
import FloatingChatButton from '../FloatingChatButton';
import FloatingChatPopup from '../FloatingChatPopup';
import styles from './styles.module.css';

type AnimationState = 'opening' | 'open' | 'closing' | 'closed';

const PREFERENCE_STORAGE_KEY = 'floating-chat-open-preference';

export default function GlobalFloatingChat(): JSX.Element | null {
  const [isOpen, setIsOpen] = useState(false);
  const [animationState, setAnimationState] = useState<AnimationState>('closed');

  // SSR compatibility check
  if (typeof window === 'undefined') {
    return null;
  }

  /**
   * Open chat popup with animation
   */
  const openChat = useCallback(() => {
    setIsOpen(true);
    setAnimationState('opening');

    // Announce to screen readers
    announceToScreenReader('AI Assistant chat opened');

    // Transition to 'open' after animation starts
    setTimeout(() => {
      setAnimationState('open');
    }, 50);

    // Save preference
    try {
      localStorage.setItem(PREFERENCE_STORAGE_KEY, 'true');
    } catch (error) {
      console.warn('Failed to save chat preference:', error);
    }
  }, []);

  /**
   * Close chat popup with animation
   */
  const closeChat = useCallback(() => {
    setAnimationState('closing');

    // Announce to screen readers
    announceToScreenReader('AI Assistant chat closed');

    // After animation completes, set isOpen to false
    setTimeout(() => {
      setIsOpen(false);
      setAnimationState('closed');
    }, 250); // Match closing animation duration

    // Save preference
    try {
      localStorage.setItem(PREFERENCE_STORAGE_KEY, 'false');
    } catch (error) {
      console.warn('Failed to save chat preference:', error);
    }
  }, []);

  /**
   * Toggle chat open/closed
   */
  const toggleChat = useCallback(() => {
    if (isOpen || animationState === 'opening') {
      closeChat();
    } else {
      openChat();
    }
  }, [isOpen, animationState, openChat, closeChat]);

  /**
   * Restore preference on mount
   */
  useEffect(() => {
    try {
      const savedPreference = localStorage.getItem(PREFERENCE_STORAGE_KEY);
      // Don't auto-open on page load (UX best practice)
      // User must explicitly click to open
      if (savedPreference === 'true') {
        // Could optionally show a small badge/notification
        console.log('[FloatingChat] Previous session was open');
      }
    } catch (error) {
      console.warn('Failed to restore chat preference:', error);
    }
  }, []);

  /**
   * Listen for custom close event from ChatbotWidget
   */
  useEffect(() => {
    const handleChatbotClose = () => {
      closeChat();
    };

    window.addEventListener('chatbot-close', handleChatbotClose);
    return () => {
      window.removeEventListener('chatbot-close', handleChatbotClose);
    };
  }, [closeChat]);

  return (
    <>
      {/* Screen reader announcement area */}
      <div
        id="floating-chat-announcer"
        role="status"
        aria-live="polite"
        aria-atomic="true"
        className={styles.visuallyHidden}
      />

      {/* Floating button (shown when closed) */}
      {!isOpen && animationState === 'closed' && (
        <FloatingChatButton onClick={openChat} isExpanded={false} />
      )}

      {/* Popup (shown when open or animating) */}
      {(isOpen || animationState === 'closing') && (
        <FloatingChatPopup
          isOpen={isOpen}
          onClose={closeChat}
          animationState={animationState}
        />
      )}
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
    // Clear after announcement
    setTimeout(() => {
      announcer.textContent = '';
    }, 1000);
  }
}
