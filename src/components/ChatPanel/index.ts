/**
 * ChatPanel Components Index
 *
 * Exports all chat panel components for Phase 6 UI enhancements.
 *
 * @see specs/007-enhance-ui/plan.md for component structure
 */

// Main component
export { ChatPanel, type ChatPanelProps } from './ChatPanel';

// Sub-components
export { ChatPanelHeader, type ChatPanelHeaderProps } from './ChatPanelHeader';
export { ChatPanelSearch, type ChatPanelSearchProps } from './ChatPanelSearch';
export { ResizeHandle, type ResizeHandleProps } from './ResizeHandle';
export { UnreadBadge, type UnreadBadgeProps } from './UnreadBadge';
export { JumpToLatest, type JumpToLatestProps } from './JumpToLatest';

// Default export
export { ChatPanel as default } from './ChatPanel';
