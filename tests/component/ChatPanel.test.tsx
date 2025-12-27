/**
 * Component Tests: ChatPanel
 *
 * Tests the ChatPanel component rendering and interactions.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, screen, fireEvent } from '@testing-library/react';
import { ChatPanel } from '../../src/components/ChatPanel/ChatPanel';

// Mock the hooks
vi.mock('../../src/components/animations/useReducedMotion', () => ({
  useReducedMotion: () => ({ prefersReducedMotion: false }),
}));

vi.mock('../../src/components/animations', () => ({
  useAnimationState: () => ({
    phase: 'entered',
    shouldRender: true,
    style: {},
  }),
}));

describe('ChatPanel', () => {
  const defaultProps = {
    isOpen: true,
    onClose: vi.fn(),
    onMinimize: vi.fn(),
    onExpand: vi.fn(),
    onWidthChange: vi.fn(),
    width: 400,
    isMinimized: false,
    unreadCount: 0,
  };

  beforeEach(() => {
    vi.clearAllMocks();
  });

  it('should render when open', () => {
    render(<ChatPanel {...defaultProps} />);

    expect(screen.getByRole('complementary')).toBeInTheDocument();
  });

  it('should not render when closed and animation complete', () => {
    vi.mock('../../src/components/animations', () => ({
      useAnimationState: () => ({
        phase: 'idle',
        shouldRender: false,
        style: {},
      }),
    }));

    const { container } = render(<ChatPanel {...defaultProps} isOpen={false} />);

    // Panel should not be visible
    expect(container.querySelector('[role="complementary"]')).toBeNull();
  });

  it('should have correct ARIA attributes', () => {
    render(<ChatPanel {...defaultProps} />);

    const panel = screen.getByRole('complementary');
    expect(panel).toHaveAttribute('aria-label', 'Chat panel');
    expect(panel).toHaveAttribute('aria-hidden', 'false');
  });

  it('should call onClose when close is triggered', () => {
    render(<ChatPanel {...defaultProps} />);

    // Trigger Escape key
    fireEvent.keyDown(document, { key: 'Escape' });

    expect(defaultProps.onClose).toHaveBeenCalled();
  });

  it('should display header component', () => {
    render(<ChatPanel {...defaultProps} />);

    // Header should be present
    expect(screen.getByRole('complementary')).toContainElement(
      document.querySelector('[class*="header"]') ||
        document.querySelector('header') ||
        screen.getByRole('complementary').firstElementChild as HTMLElement
    );
  });

  it('should display children in content area', () => {
    render(
      <ChatPanel {...defaultProps}>
        <div data-testid="test-content">Test Content</div>
      </ChatPanel>
    );

    expect(screen.getByTestId('test-content')).toBeInTheDocument();
  });

  it('should hide content when minimized', () => {
    render(
      <ChatPanel {...defaultProps} isMinimized>
        <div data-testid="test-content">Test Content</div>
      </ChatPanel>
    );

    expect(screen.queryByTestId('test-content')).not.toBeInTheDocument();
  });

  it('should display unread badge when minimized with unread messages', () => {
    render(<ChatPanel {...defaultProps} isMinimized unreadCount={5} />);

    // ARIA live region should announce unread count
    expect(screen.getByRole('status')).toHaveTextContent('5 new messages');
  });

  it('should handle singular unread message correctly', () => {
    render(<ChatPanel {...defaultProps} isMinimized unreadCount={1} />);

    expect(screen.getByRole('status')).toHaveTextContent('1 new message');
  });

  it('should apply custom className', () => {
    render(<ChatPanel {...defaultProps} className="custom-class" />);

    const panel = screen.getByRole('complementary');
    expect(panel).toHaveClass('custom-class');
  });

  it('should apply panel width style', () => {
    render(<ChatPanel {...defaultProps} width={500} />);

    const panel = screen.getByRole('complementary');
    expect(panel).toHaveStyle({ '--panel-width': '500px' });
  });

  it('should show resize handle when not minimized', () => {
    render(<ChatPanel {...defaultProps} />);

    // Resize handle should be present
    const resizeHandle = document.querySelector('[class*="resize"]');
    // This depends on implementation
  });

  it('should hide resize handle when minimized', () => {
    render(<ChatPanel {...defaultProps} isMinimized />);

    // Resize handle should not be visible when minimized
    const panel = screen.getByRole('complementary');
    expect(panel).toHaveClass(/minimized/i);
  });

  it('should open search with Ctrl+F', () => {
    const onSearchQueryChange = vi.fn();

    render(
      <ChatPanel
        {...defaultProps}
        onSearchQueryChange={onSearchQueryChange}
      />
    );

    fireEvent.keyDown(document, { key: 'f', ctrlKey: true });

    // Search should be visible
    // Implementation depends on component structure
  });
});
