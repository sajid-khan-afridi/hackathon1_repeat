/**
 * useChatPanelState Hook
 *
 * Manages the complete state of the docked chat panel including
 * visibility, minimization, width, and search functionality.
 *
 * @see specs/007-enhance-ui/data-model.md for ChatPanelState interface
 */

import { useState, useCallback, useEffect, useMemo } from 'react';
import { usePersistedState } from './usePersistedState';

/**
 * Storage keys for persisted state
 */
const STORAGE_KEYS = {
  CHAT_PANEL_OPEN: 'robotics-textbook:chat-panel-open',
  CHAT_PANEL_WIDTH: 'robotics-textbook:chat-panel-width',
  CHAT_PANEL_MINIMIZED: 'robotics-textbook:chat-panel-minimized',
} as const;

/**
 * Panel width constraints
 */
export const PANEL_WIDTH_CONSTRAINTS = {
  MIN: 320,
  MAX: 600,
  DEFAULT: 400,
  STEP: 10,
} as const;

/**
 * Chat panel state interface
 */
export interface ChatPanelState {
  /** Whether the panel is open (visible) */
  isOpen: boolean;

  /** Whether the panel is collapsed to minimized bar */
  isMinimized: boolean;

  /** Panel width in pixels (320-600 range) */
  width: number;

  /** Number of unread messages when panel is closed/minimized */
  unreadCount: number;

  /** Current scroll position in message list (pixels from bottom) */
  scrollPosition: number;

  /** Search query in chat history */
  searchQuery: string;

  /** Indices of messages matching search query */
  searchResults: number[];

  /** Index of currently highlighted search result */
  activeSearchIndex: number;
}

/**
 * Return type for useChatPanelState hook
 */
export interface UseChatPanelStateReturn {
  /** Current state */
  state: ChatPanelState;

  /** Open the panel */
  open: () => void;

  /** Close the panel */
  close: () => void;

  /** Toggle panel open/close */
  toggle: () => void;

  /** Minimize the panel */
  minimize: () => void;

  /** Expand from minimized */
  expand: () => void;

  /** Set panel width */
  setWidth: (width: number) => void;

  /** Increment unread count */
  addUnread: () => void;

  /** Clear unread count */
  clearUnread: () => void;

  /** Set search query */
  setSearchQuery: (query: string) => void;

  /** Navigate search results */
  nextSearchResult: () => void;
  prevSearchResult: () => void;

  /** Clear search */
  clearSearch: () => void;

  /** Set search results from message matching */
  setSearchResults: (results: number[]) => void;

  /** Update scroll position */
  setScrollPosition: (position: number) => void;
}

/**
 * Validates and clamps panel width to allowed range
 */
function validatePanelWidth(width: number): number {
  return Math.max(PANEL_WIDTH_CONSTRAINTS.MIN, Math.min(PANEL_WIDTH_CONSTRAINTS.MAX, width));
}

/**
 * Custom hook for managing chat panel state.
 *
 * Features:
 * - Persistent state across page refreshes (localStorage/sessionStorage)
 * - Falls back to in-memory storage when browser storage unavailable
 * - Width validation (320-600px range)
 * - Search state management
 *
 * @example
 * ```tsx
 * function ChatPanel() {
 *   const {
 *     state,
 *     open,
 *     close,
 *     minimize,
 *     expand,
 *     setWidth,
 *   } = useChatPanelState();
 *
 *   return (
 *     <div style={{ width: state.width }}>
 *       {state.isOpen && !state.isMinimized && <ChatContent />}
 *     </div>
 *   );
 * }
 * ```
 */
export function useChatPanelState(): UseChatPanelStateReturn {
  // Persisted state: isOpen (survives refresh)
  const [isOpen, setIsOpen] = usePersistedState<boolean>(STORAGE_KEYS.CHAT_PANEL_OPEN, false, {
    storage: 'localStorage',
  });

  // Persisted state: width (user preference)
  const [width, setWidthState] = usePersistedState<number>(
    STORAGE_KEYS.CHAT_PANEL_WIDTH,
    PANEL_WIDTH_CONSTRAINTS.DEFAULT,
    { storage: 'localStorage' }
  );

  // Session state: isMinimized (reset on new session)
  const [isMinimized, setIsMinimized] = usePersistedState<boolean>(
    STORAGE_KEYS.CHAT_PANEL_MINIMIZED,
    false,
    { storage: 'sessionStorage' }
  );

  // Memory-only state (reset on close)
  const [unreadCount, setUnreadCount] = useState(0);
  const [scrollPosition, setScrollPosition] = useState(0);
  const [searchQuery, setSearchQueryState] = useState('');
  const [searchResults, setSearchResultsState] = useState<number[]>([]);
  const [activeSearchIndex, setActiveSearchIndex] = useState(-1);

  // Clear unread when panel opens
  useEffect(() => {
    if (isOpen && !isMinimized) {
      setUnreadCount(0);
    }
  }, [isOpen, isMinimized]);

  // Reset active search index when results change
  useEffect(() => {
    if (searchResults.length > 0) {
      setActiveSearchIndex(0);
    } else {
      setActiveSearchIndex(-1);
    }
  }, [searchResults]);

  // State object
  const state = useMemo<ChatPanelState>(
    () => ({
      isOpen,
      isMinimized,
      width,
      unreadCount,
      scrollPosition,
      searchQuery,
      searchResults,
      activeSearchIndex,
    }),
    [
      isOpen,
      isMinimized,
      width,
      unreadCount,
      scrollPosition,
      searchQuery,
      searchResults,
      activeSearchIndex,
    ]
  );

  // Actions
  const open = useCallback(() => {
    setIsOpen(true);
    setIsMinimized(false);
  }, [setIsOpen, setIsMinimized]);

  const close = useCallback(() => {
    setIsOpen(false);
    // Reset memory state on close
    setSearchQueryState('');
    setSearchResultsState([]);
    setActiveSearchIndex(-1);
    setScrollPosition(0);
  }, [setIsOpen]);

  const toggle = useCallback(() => {
    if (isOpen) {
      close();
    } else {
      open();
    }
  }, [isOpen, open, close]);

  const minimize = useCallback(() => {
    setIsMinimized(true);
  }, [setIsMinimized]);

  const expand = useCallback(() => {
    setIsMinimized(false);
    setUnreadCount(0);
  }, [setIsMinimized]);

  const setWidth = useCallback(
    (newWidth: number) => {
      setWidthState(validatePanelWidth(newWidth));
    },
    [setWidthState]
  );

  const addUnread = useCallback(() => {
    if (isMinimized || !isOpen) {
      setUnreadCount((prev) => prev + 1);
    }
  }, [isMinimized, isOpen]);

  const clearUnread = useCallback(() => {
    setUnreadCount(0);
  }, []);

  const setSearchQuery = useCallback((query: string) => {
    setSearchQueryState(query.slice(0, 100).trim());
  }, []);

  const setSearchResults = useCallback((results: number[]) => {
    setSearchResultsState(results);
  }, []);

  const clearSearch = useCallback(() => {
    setSearchQueryState('');
    setSearchResultsState([]);
    setActiveSearchIndex(-1);
  }, []);

  const nextSearchResult = useCallback(() => {
    if (searchResults.length === 0) return;
    setActiveSearchIndex((prev) => (prev >= searchResults.length - 1 ? 0 : prev + 1));
  }, [searchResults.length]);

  const prevSearchResult = useCallback(() => {
    if (searchResults.length === 0) return;
    setActiveSearchIndex((prev) => (prev <= 0 ? searchResults.length - 1 : prev - 1));
  }, [searchResults.length]);

  return {
    state,
    open,
    close,
    toggle,
    minimize,
    expand,
    setWidth,
    addUnread,
    clearUnread,
    setSearchQuery,
    nextSearchResult,
    prevSearchResult,
    clearSearch,
    setSearchResults,
    setScrollPosition,
  };
}

export default useChatPanelState;
