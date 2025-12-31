/**
 * ChatPanel Component
 *
 * Docked chat panel with resize, minimize, search, and smooth animations.
 * Main container component that composes header, search, and content areas.
 *
 * @see specs/007-enhance-ui/plan.md for architecture
 * @see specs/007-enhance-ui/data-model.md for state interfaces
 */

import React, { useRef, useEffect, useCallback, useState } from 'react';
import clsx from 'clsx';
import { useAnimationState } from '../animations';
import { ChatPanelHeader } from './ChatPanelHeader';
import { ResizeHandle } from './ResizeHandle';
import { ChatPanelSearch } from './ChatPanelSearch';
import { PANEL_WIDTH_CONSTRAINTS } from '../../hooks/useChatPanelState';
import styles from './ChatPanel.module.css';

export interface ChatPanelProps {
  /** Whether panel is open */
  isOpen: boolean;

  /** Callback when panel should close */
  onClose: () => void;

  /** Callback when panel should minimize */
  onMinimize: () => void;

  /** Callback when panel should expand */
  onExpand: () => void;

  /** Callback when width changes */
  onWidthChange: (width: number) => void;

  /** Current width in pixels */
  width: number;

  /** Whether panel is minimized */
  isMinimized: boolean;

  /** Unread message count */
  unreadCount: number;

  /** Children to render in the content area */
  children?: React.ReactNode;

  /** Search query */
  searchQuery?: string;

  /** Search result count */
  searchResultCount?: number;

  /** Active search result index */
  searchActiveIndex?: number;

  /** Callback when search query changes */
  onSearchQueryChange?: (query: string) => void;

  /** Callback for next search result */
  onSearchNext?: () => void;

  /** Callback for previous search result */
  onSearchPrev?: () => void;

  /** Callback to close search */
  onSearchClose?: () => void;

  /** Additional CSS class */
  className?: string;
}

/**
 * ChatPanel Component
 *
 * A docked chat panel that can be resized, minimized, and searched.
 * Supports RTL layouts and respects reduced motion preferences.
 */
export function ChatPanel({
  isOpen,
  onClose,
  onMinimize,
  onExpand,
  onWidthChange,
  width,
  isMinimized,
  unreadCount,
  children,
  searchQuery = '',
  searchResultCount = 0,
  searchActiveIndex = 0,
  onSearchQueryChange,
  onSearchNext,
  onSearchPrev,
  onSearchClose,
  className,
}: ChatPanelProps): React.ReactElement | null {
  const panelRef = useRef<HTMLDivElement>(null);
  const previousFocusRef = useRef<HTMLElement | null>(null);
  const [isSearchVisible, setIsSearchVisible] = useState(false);
  const [isRTL, setIsRTL] = useState(false);
  const [isMobile, setIsMobile] = useState(false);

  // Detect RTL mode
  useEffect(() => {
    if (typeof document !== 'undefined') {
      setIsRTL(document.documentElement.dir === 'rtl');
    }
  }, []);

  // Handle viewport resize - adjust panel behavior for mobile
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const checkMobile = () => {
      setIsMobile(window.innerWidth < 768);
    };

    // Initial check
    checkMobile();

    // Debounced resize handler to prevent animation conflicts
    let resizeTimeout: ReturnType<typeof setTimeout>;
    const handleResize = () => {
      clearTimeout(resizeTimeout);
      resizeTimeout = setTimeout(checkMobile, 100);
    };

    window.addEventListener('resize', handleResize);
    return () => {
      window.removeEventListener('resize', handleResize);
      clearTimeout(resizeTimeout);
    };
  }, []);

  // Animation state
  const {
    phase,
    shouldRender,
    style: animationStyle,
  } = useAnimationState({
    isVisible: isOpen,
    enterDuration: 300,
    exitDuration: 200,
  });

  // Focus management: save focus on open, restore on close
  useEffect(() => {
    if (isOpen && panelRef.current) {
      previousFocusRef.current = document.activeElement as HTMLElement;
      panelRef.current.focus();
    } else if (!isOpen && previousFocusRef.current) {
      previousFocusRef.current.focus();
      previousFocusRef.current = null;
    }
  }, [isOpen]);

  // Keyboard shortcuts
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Ctrl+F to open search
      if (e.ctrlKey && e.key === 'f' && isOpen && !isMinimized) {
        e.preventDefault();
        setIsSearchVisible(true);
      }

      // Escape to close panel (if search is closed)
      if (e.key === 'Escape' && isOpen && !isSearchVisible) {
        onClose();
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isOpen, isMinimized, isSearchVisible, onClose]);

  // Handle search toggle
  const handleSearchToggle = useCallback(() => {
    setIsSearchVisible((prev) => !prev);
  }, []);

  // Handle search close
  const handleSearchClose = useCallback(() => {
    setIsSearchVisible(false);
    onSearchClose?.();
  }, [onSearchClose]);

  // Don't render if not needed
  if (!shouldRender) {
    return null;
  }

  // Build panel style - use full width on mobile, custom width on desktop
  const panelStyle: React.CSSProperties = {
    '--panel-width': isMobile ? '100%' : `${width}px`,
    ...animationStyle,
  } as React.CSSProperties;

  return (
    <div
      ref={panelRef}
      className={clsx(
        styles.panel,
        styles[phase],
        {
          [styles.panelMinimized]: isMinimized,
        },
        className
      )}
      style={panelStyle}
      role="complementary"
      aria-label="Chat panel"
      aria-hidden={!isOpen}
      tabIndex={-1}
    >
      {/* Resize Handle - hide on mobile */}
      {!isMinimized && !isMobile && (
        <ResizeHandle
          currentWidth={width}
          minWidth={PANEL_WIDTH_CONSTRAINTS.MIN}
          maxWidth={PANEL_WIDTH_CONSTRAINTS.MAX}
          onResize={onWidthChange}
          isRTL={isRTL}
        />
      )}

      {/* Header */}
      <ChatPanelHeader
        isMinimized={isMinimized}
        isSearchActive={isSearchVisible}
        onMinimize={onMinimize}
        onExpand={onExpand}
        onClose={onClose}
        onSearchToggle={handleSearchToggle}
      />

      {/* Search (only when not minimized) */}
      {!isMinimized && onSearchQueryChange && (
        <ChatPanelSearch
          isVisible={isSearchVisible}
          query={searchQuery}
          onQueryChange={onSearchQueryChange}
          activeIndex={searchActiveIndex}
          resultCount={searchResultCount}
          onNextResult={onSearchNext || (() => {})}
          onPrevResult={onSearchPrev || (() => {})}
          onClose={handleSearchClose}
        />
      )}

      {/* Content Area (only when not minimized) */}
      {!isMinimized && (
        <div className={styles.content}>
          <div className={styles.messageArea}>{children}</div>
        </div>
      )}

      {/* ARIA Live Region for state changes */}
      <div
        role="status"
        aria-live="polite"
        aria-atomic="true"
        className="sr-only"
        style={{ position: 'absolute', left: '-9999px' }}
      >
        {isMinimized && unreadCount > 0 && (
          <span>{`${unreadCount} new message${unreadCount === 1 ? '' : 's'}`}</span>
        )}
      </div>
    </div>
  );
}

export default ChatPanel;
