# Phase 1: Data Model - UI Enhancement

**Feature Branch**: `007-enhance-ui`
**Date**: 2025-12-27
**Status**: Complete

## Overview

This document defines the TypeScript interfaces and state management patterns for Phase 6 UI enhancements. Since this is a UI-only phase, there are no database schema changes - all models are client-side component state.

---

## Core Entities

### 1. ChatPanelState

Represents the complete state of the docked chat panel.

```typescript
/**
 * Chat panel state interface
 * Persistence: Hybrid (localStorage/sessionStorage/memory)
 */
interface ChatPanelState {
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
 * Default chat panel state
 */
const DEFAULT_CHAT_PANEL_STATE: ChatPanelState = {
  isOpen: false,
  isMinimized: false,
  width: 400,
  unreadCount: 0,
  scrollPosition: 0,
  searchQuery: '',
  searchResults: [],
  activeSearchIndex: -1,
};

/**
 * Persistence strategy for each field
 */
const CHAT_PANEL_PERSISTENCE: Record<keyof ChatPanelState, 'localStorage' | 'sessionStorage' | 'memory'> = {
  isOpen: 'localStorage',        // Survives page refresh
  isMinimized: 'sessionStorage', // Reset on new session
  width: 'localStorage',         // User preference preserved
  unreadCount: 'memory',         // Reset on close
  scrollPosition: 'memory',      // Reset on close
  searchQuery: 'memory',         // Reset on close
  searchResults: 'memory',       // Derived from searchQuery
  activeSearchIndex: 'memory',   // Reset on close
};

/**
 * Storage keys for persisted state
 */
const STORAGE_KEYS = {
  CHAT_PANEL_OPEN: 'robotics-textbook:chat-panel-open',
  CHAT_PANEL_WIDTH: 'robotics-textbook:chat-panel-width',
  CHAT_PANEL_MINIMIZED: 'robotics-textbook:chat-panel-minimized',
} as const;
```

### 2. AnimationPreference

Represents user's motion preference from system settings.

```typescript
/**
 * Animation preference derived from system settings
 * Read-only: Cannot be overridden by user (respects OS preference)
 */
interface AnimationPreference {
  /** Whether user prefers reduced motion */
  prefersReducedMotion: boolean;

  /** Actual animation duration multiplier (0-1, 0 = instant) */
  durationMultiplier: number;

  /** Whether to use movement animations (translateX/Y) */
  allowMovement: boolean;

  /** Whether to use opacity animations */
  allowOpacity: boolean;
}

/**
 * Full motion preference (default)
 */
const FULL_MOTION: AnimationPreference = {
  prefersReducedMotion: false,
  durationMultiplier: 1,
  allowMovement: true,
  allowOpacity: true,
};

/**
 * Reduced motion preference
 */
const REDUCED_MOTION: AnimationPreference = {
  prefersReducedMotion: true,
  durationMultiplier: 0.01,
  allowMovement: false,
  allowOpacity: true, // Opacity-only transitions still allowed
};
```

### 3. NavbarItem

Represents a navigation item with icon and tooltip.

```typescript
/**
 * Navbar item with icon configuration
 */
interface NavbarItem {
  /** Unique identifier */
  id: string;

  /** Display label */
  label: string;

  /** Lucide icon name (e.g., 'Book', 'MessageSquare') */
  iconName: string;

  /** Navigation link */
  href: string;

  /** Whether this is the currently active item */
  isActive: boolean;

  /** Whether to show in mobile view */
  showOnMobile: boolean;

  /** Dropdown children (if any) */
  children?: NavbarItem[];

  /** External link (opens in new tab) */
  isExternal?: boolean;
}

/**
 * Tooltip configuration for navbar icons
 */
interface NavbarTooltipConfig {
  /** Tooltip text content */
  content: string;

  /** Placement relative to icon */
  placement: 'top' | 'bottom' | 'left' | 'right';

  /** Delay before showing (ms) */
  delayShow: number;

  /** Delay before hiding (ms) */
  delayHide: number;
}

const DEFAULT_TOOLTIP_CONFIG: NavbarTooltipConfig = {
  content: '',
  placement: 'bottom',
  delayShow: 300,
  delayHide: 0,
};
```

### 4. ScrollRevealState

Represents state for scroll-triggered animations.

```typescript
/**
 * Scroll reveal animation state
 */
interface ScrollRevealState {
  /** Whether element is visible in viewport */
  isVisible: boolean;

  /** Visibility percentage (0-1) */
  visibilityRatio: number;

  /** Whether animation has been triggered (for once-only animations) */
  hasTriggered: boolean;

  /** Reference to DOM element */
  ref: React.RefObject<HTMLElement>;
}

/**
 * Scroll reveal options
 */
interface ScrollRevealOptions {
  /** Visibility threshold to trigger (0-1, default 0.2) */
  threshold?: number;

  /** Whether to animate only once (default true) */
  once?: boolean;

  /** Root margin for IntersectionObserver */
  rootMargin?: string;

  /** Animation delay in ms */
  delay?: number;
}
```

### 5. LoadingState

Represents various loading states for UI feedback.

```typescript
/**
 * Loading state types
 */
type LoadingType = 'skeleton' | 'spinner' | 'typing' | 'progress';

/**
 * Loading state configuration
 */
interface LoadingState {
  /** Whether currently loading */
  isLoading: boolean;

  /** Type of loading indicator to show */
  type: LoadingType;

  /** Progress percentage (0-100, only for 'progress' type) */
  progress?: number;

  /** Loading message for screen readers */
  ariaLabel: string;

  /** Estimated time remaining in ms (for progress bars) */
  estimatedTime?: number;
}

/**
 * Loading thresholds (when to show which type)
 */
const LOADING_THRESHOLDS = {
  /** Show skeleton for content > 200ms */
  SKELETON_DELAY: 200,
  /** Show progress bar for operations > 3s */
  PROGRESS_THRESHOLD: 3000,
  /** Show spinner for quick operations < 3s */
  SPINNER_MAX: 3000,
} as const;
```

### 6. MessageAnimation

Represents animation state for chat messages.

```typescript
/**
 * Animation state for individual messages
 */
interface MessageAnimationState {
  /** Animation phase */
  phase: 'entering' | 'entered' | 'exiting' | 'exited';

  /** Stagger delay based on position (ms) */
  staggerDelay: number;

  /** Whether to apply slide animation */
  shouldSlide: boolean;

  /** Whether to apply fade animation */
  shouldFade: boolean;
}

/**
 * Message animation configuration
 */
const MESSAGE_ANIMATION_CONFIG = {
  /** Duration for message entrance */
  ENTER_DURATION: 300,
  /** Duration for message exit */
  EXIT_DURATION: 200,
  /** Stagger delay between consecutive messages */
  STAGGER_DELAY: 50,
  /** Maximum stagger delay */
  MAX_STAGGER: 200,
} as const;
```

---

## Component Props Interfaces

### ChatPanel Component

```typescript
interface ChatPanelProps {
  /** Whether panel is open */
  isOpen: boolean;

  /** Callback when panel should close */
  onClose: () => void;

  /** Callback when panel should minimize */
  onMinimize: () => void;

  /** Callback when width changes */
  onWidthChange: (width: number) => void;

  /** Current width in pixels */
  width: number;

  /** Whether panel is minimized */
  isMinimized: boolean;

  /** Unread message count */
  unreadCount: number;

  /** RTL mode (panel docks to left) */
  isRTL?: boolean;

  /** Class name for custom styling */
  className?: string;
}
```

### ResizeHandle Component

```typescript
interface ResizeHandleProps {
  /** Callback during resize */
  onResize: (width: number) => void;

  /** Callback when resize starts */
  onResizeStart?: () => void;

  /** Callback when resize ends */
  onResizeEnd?: () => void;

  /** Current panel width */
  currentWidth: number;

  /** Minimum allowed width */
  minWidth: number;

  /** Maximum allowed width */
  maxWidth: number;

  /** RTL mode (handle on right edge instead of left) */
  isRTL?: boolean;
}
```

### UnreadBadge Component

```typescript
interface UnreadBadgeProps {
  /** Number of unread messages */
  count: number;

  /** Maximum count to display (shows "99+" for higher) */
  maxCount?: number;

  /** Whether to show pulse animation */
  animate?: boolean;

  /** Size variant */
  size?: 'sm' | 'md' | 'lg';
}
```

### JumpToLatest Component

```typescript
interface JumpToLatestProps {
  /** Whether button is visible */
  visible: boolean;

  /** Callback when clicked */
  onClick: () => void;

  /** Number of messages below current view */
  newMessageCount?: number;
}
```

### ScrollReveal Component

```typescript
interface ScrollRevealProps {
  /** Child elements to reveal */
  children: React.ReactNode;

  /** Animation type */
  animation?: 'fade-up' | 'fade-in' | 'slide-left' | 'slide-right' | 'scale';

  /** Stagger children animations */
  stagger?: boolean;

  /** Stagger delay in ms */
  staggerDelay?: number;

  /** Visibility threshold (0-1) */
  threshold?: number;

  /** Animate only once */
  once?: boolean;

  /** Animation duration in ms */
  duration?: number;

  /** Animation delay in ms */
  delay?: number;

  /** Class name for container */
  className?: string;
}
```

### SkeletonLoader Component

```typescript
interface SkeletonLoaderProps {
  /** Width of skeleton (CSS value or 'full') */
  width?: string | 'full';

  /** Height of skeleton (CSS value) */
  height?: string;

  /** Border radius */
  borderRadius?: string;

  /** Whether to show shimmer animation */
  animate?: boolean;

  /** Number of skeleton lines (for text) */
  lines?: number;

  /** Variant type */
  variant?: 'text' | 'circular' | 'rectangular';
}
```

### TypingIndicator Component

```typescript
interface TypingIndicatorProps {
  /** Whether indicator is visible */
  visible: boolean;

  /** Optional label for who is typing */
  label?: string;

  /** Dot count */
  dotCount?: number;
}
```

---

## Hook Interfaces

### useChatPanelState

```typescript
interface UseChatPanelStateReturn {
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
}
```

### useReducedMotion

```typescript
interface UseReducedMotionReturn {
  /** Whether user prefers reduced motion */
  prefersReducedMotion: boolean;

  /** Get animation duration adjusted for preference */
  getAdjustedDuration: (baseDuration: number) => number;

  /** Whether to skip animation entirely */
  shouldSkipAnimation: boolean;
}
```

### useScrollReveal

```typescript
interface UseScrollRevealReturn {
  /** Ref to attach to element */
  ref: React.RefObject<HTMLElement>;

  /** Whether element is visible */
  isVisible: boolean;

  /** CSS class names for animation state */
  className: string;
}
```

### usePersistedState

```typescript
interface UsePersistedStateOptions<T> {
  /** Storage type */
  storage: 'localStorage' | 'sessionStorage';

  /** Serialize function */
  serialize?: (value: T) => string;

  /** Deserialize function */
  deserialize?: (value: string) => T;
}

type UsePersistedStateReturn<T> = [T, (value: T | ((prev: T) => T)) => void];
```

---

## Validation Rules

### Panel Width Validation

```typescript
const PANEL_WIDTH_CONSTRAINTS = {
  MIN: 320,
  MAX: 600,
  DEFAULT: 400,
  STEP: 10, // For keyboard resize
} as const;

function validatePanelWidth(width: number): number {
  return Math.max(
    PANEL_WIDTH_CONSTRAINTS.MIN,
    Math.min(PANEL_WIDTH_CONSTRAINTS.MAX, width)
  );
}
```

### Search Query Validation

```typescript
const SEARCH_CONSTRAINTS = {
  MIN_LENGTH: 2,
  MAX_LENGTH: 100,
  DEBOUNCE_MS: 300,
} as const;

function validateSearchQuery(query: string): string {
  return query.slice(0, SEARCH_CONSTRAINTS.MAX_LENGTH).trim();
}
```

---

## State Transitions

### Panel State Machine

```
┌──────────┐   open()    ┌────────┐   minimize()   ┌───────────┐
│  CLOSED  │ ──────────> │  OPEN  │ ─────────────> │ MINIMIZED │
└──────────┘             └────────┘                └───────────┘
     ^                        │                          │
     │                        │ close()                  │
     │                        v                          │ expand()
     │                   ┌────────┐                      │
     └─────────────────  │ CLOSED │ <────────────────────┘
                         └────────┘           close()
```

### Animation State Machine

```
┌─────────┐   trigger   ┌──────────┐   complete   ┌─────────┐
│ INITIAL │ ──────────> │ ENTERING │ ───────────> │ ENTERED │
└─────────┘             └──────────┘              └─────────┘
                                                       │
                                                       │ exit()
                                                       v
                                                  ┌──────────┐
                                                  │ EXITING  │
                                                  └──────────┘
                                                       │
                                                       │ complete
                                                       v
                                                  ┌──────────┐
                                                  │  EXITED  │
                                                  └──────────┘
```

---

## Summary

Key entities defined:
1. **ChatPanelState** - Complete panel state with persistence strategy
2. **AnimationPreference** - System-derived motion preferences
3. **NavbarItem** - Navigation item with icon and tooltip config
4. **ScrollRevealState** - Scroll-triggered animation state
5. **LoadingState** - Loading indicator configurations
6. **MessageAnimationState** - Per-message animation state

All interfaces are TypeScript-first with strict typing. No database schema changes required - this is purely client-side state management.
