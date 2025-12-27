/**
 * GlobalFloatingChat (Enhanced)
 *
 * Enhanced global wrapper for the floating chat using the new ChatPanel component.
 * Replaces the popup modal with a docked panel design.
 *
 * Features:
 * - Docked panel (right side, left in RTL)
 * - Resizable width (320-600px)
 * - Minimize to bar
 * - Search in chat history
 * - Unread message badge
 * - Focus management
 * - Keyboard shortcuts (Ctrl+F for search, Escape to close)
 * - localStorage persistence for panel state
 *
 * @see specs/007-enhance-ui/plan.md for architecture
 */

import React, { useEffect, useCallback, useState } from 'react';
import FloatingChatButton from '../FloatingChatButton';
import { ChatPanel } from '../ChatPanel';
import { UnreadBadge } from '../ChatPanel/UnreadBadge';
import { useChatPanelState } from '../../hooks/useChatPanelState';
import styles from './GlobalFloatingChat.module.css';

/**
 * GlobalFloatingChat Component
 *
 * Enhanced chat interface with docked panel design.
 */
export default function GlobalFloatingChat(): React.ReactElement | null {
  const {
    state,
    open,
    close,
    minimize,
    expand,
    setWidth,
    setSearchQuery,
    nextSearchResult,
    prevSearchResult,
    clearSearch,
    setSearchResults,
  } = useChatPanelState();

  // Track if we're on the client
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);
  }, []);

  // SSR compatibility
  if (!isClient) {
    return null;
  }

  // Announce to screen readers
  const announceToScreenReader = useCallback((message: string) => {
    const announcer = document.getElementById('floating-chat-announcer');
    if (announcer) {
      announcer.textContent = message;
      setTimeout(() => {
        announcer.textContent = '';
      }, 1000);
    }
  }, []);

  // Handle open with announcement
  const handleOpen = useCallback(() => {
    open();
    announceToScreenReader('AI Assistant chat opened');
  }, [open, announceToScreenReader]);

  // Handle close with announcement
  const handleClose = useCallback(() => {
    close();
    announceToScreenReader('AI Assistant chat closed');
  }, [close, announceToScreenReader]);

  // Handle minimize with announcement
  const handleMinimize = useCallback(() => {
    minimize();
    announceToScreenReader('AI Assistant minimized');
  }, [minimize, announceToScreenReader]);

  // Handle expand with announcement
  const handleExpand = useCallback(() => {
    expand();
    announceToScreenReader('AI Assistant expanded');
  }, [expand, announceToScreenReader]);

  // Listen for custom close event from ChatbotWidget
  useEffect(() => {
    const handleChatbotClose = () => {
      handleClose();
    };

    window.addEventListener('chatbot-close', handleChatbotClose);
    return () => {
      window.removeEventListener('chatbot-close', handleChatbotClose);
    };
  }, [handleClose]);

  // Simple search implementation (can be enhanced to search actual messages)
  const handleSearchQueryChange = useCallback(
    (query: string) => {
      setSearchQuery(query);
      // TODO: Implement actual message search
      // For now, just update the query
      if (query.length >= 2) {
        // Placeholder: in real implementation, search through messages
        setSearchResults([]);
      } else {
        setSearchResults([]);
      }
    },
    [setSearchQuery, setSearchResults]
  );

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

      {/* Floating button (shown when panel is closed) */}
      {!state.isOpen && (
        <div className={styles.buttonWrapper}>
          <FloatingChatButton onClick={handleOpen} isExpanded={false} />
          {state.unreadCount > 0 && (
            <UnreadBadge
              count={state.unreadCount}
              className={styles.unreadBadge}
            />
          )}
        </div>
      )}

      {/* Chat Panel (docked, resizable) */}
      <ChatPanel
        isOpen={state.isOpen}
        isMinimized={state.isMinimized}
        width={state.width}
        unreadCount={state.unreadCount}
        onClose={handleClose}
        onMinimize={handleMinimize}
        onExpand={handleExpand}
        onWidthChange={setWidth}
        searchQuery={state.searchQuery}
        searchResultCount={state.searchResults.length}
        searchActiveIndex={state.activeSearchIndex}
        onSearchQueryChange={handleSearchQueryChange}
        onSearchNext={nextSearchResult}
        onSearchPrev={prevSearchResult}
        onSearchClose={clearSearch}
      >
        {/* Chat content - use existing ChatbotWidget */}
        <div className={styles.chatContent}>
          {/* Import ChatbotWidget dynamically to avoid circular dependency */}
          <ChatbotWidgetLazy />
        </div>
      </ChatPanel>
    </>
  );
}

/**
 * Lazy-loaded ChatbotWidget to avoid circular dependencies
 */
function ChatbotWidgetLazy(): React.ReactElement {
  const [ChatbotWidget, setChatbotWidget] = useState<React.ComponentType | null>(null);

  useEffect(() => {
    import('../ChatbotWidget').then((module) => {
      setChatbotWidget(() => module.default);
    });
  }, []);

  if (!ChatbotWidget) {
    return <div className={styles.loading}>Loading...</div>;
  }

  return <ChatbotWidget />;
}
