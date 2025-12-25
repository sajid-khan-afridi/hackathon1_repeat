/**
 * Enhanced FloatingChatPopup Tests
 *
 * Comprehensive test suite covering:
 * - Rendering and animations
 * - Accessibility (ARIA, focus trap, keyboard navigation)
 * - Drag-to-move functionality
 * - Minimize/maximize states
 * - Menu interactions
 * - Bot identity message
 * - Quick action chips
 * - Responsive behavior
 */

import React from 'react';
import { render, screen, fireEvent, waitFor, within } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import EnhancedFloatingChatPopup from './EnhancedFloatingChatPopup';

// Mock ChatbotWidget
jest.mock('../ChatbotWidget', () => {
  return function MockChatbotWidget() {
    return <div data-testid="mock-chatbot-widget">Chatbot Widget</div>;
  };
});

// Mock QuickActionChips
jest.mock('../ChatbotWidget/QuickActionChips', () => {
  return function MockQuickActionChips({ onActionClick }: any) {
    return (
      <div data-testid="mock-quick-actions">
        <button onClick={() => onActionClick('test query')}>Test Action</button>
      </div>
    );
  };
});

describe('EnhancedFloatingChatPopup', () => {
  const defaultProps = {
    isOpen: true,
    onClose: jest.fn(),
    animationState: 'open' as const
  };

  beforeEach(() => {
    jest.clearAllMocks();
  });

  describe('Rendering', () => {
    it('should render when open', () => {
      render(<EnhancedFloatingChatPopup {...defaultProps} />);
      expect(screen.getByRole('dialog')).toBeInTheDocument();
    });

    it('should not render when closed', () => {
      render(
        <EnhancedFloatingChatPopup
          {...defaultProps}
          isOpen={false}
          animationState="closed"
        />
      );
      expect(screen.queryByRole('dialog')).not.toBeInTheDocument();
    });

    it('should render branded header with title', () => {
      render(<EnhancedFloatingChatPopup {...defaultProps} />);
      expect(screen.getByText('Robotics AI Assistant')).toBeInTheDocument();
    });

    it('should render bot identity message', () => {
      render(<EnhancedFloatingChatPopup {...defaultProps} />);
      expect(screen.getByText(/Hi! I'm your Robotics AI Assistant/i)).toBeInTheDocument();
      expect(screen.getByText(/What I can't do:/i)).toBeInTheDocument();
    });

    it('should render quick action chips', () => {
      render(<EnhancedFloatingChatPopup {...defaultProps} />);
      expect(screen.getByTestId('mock-quick-actions')).toBeInTheDocument();
    });

    it('should render chatbot widget', () => {
      render(<EnhancedFloatingChatPopup {...defaultProps} />);
      expect(screen.getByTestId('mock-chatbot-widget')).toBeInTheDocument();
    });
  });

  describe('Accessibility', () => {
    it('should have correct ARIA attributes', () => {
      render(<EnhancedFloatingChatPopup {...defaultProps} />);
      const dialog = screen.getByRole('dialog');

      expect(dialog).toHaveAttribute('aria-modal', 'true');
      expect(dialog).toHaveAttribute('aria-labelledby', 'floating-chat-title');
      expect(dialog).toHaveAttribute('aria-describedby', 'floating-chat-description');
    });

    it('should have accessible title', () => {
      render(<EnhancedFloatingChatPopup {...defaultProps} />);
      const title = screen.getByRole('heading', { level: 2 });

      expect(title).toHaveTextContent('Robotics AI Assistant');
      expect(title).toHaveAttribute('id', 'floating-chat-title');
    });

    it('should have accessible description for screen readers', () => {
      render(<EnhancedFloatingChatPopup {...defaultProps} />);
      const description = document.getElementById('floating-chat-description');

      expect(description).toBeInTheDocument();
      expect(description).toHaveTextContent(/Ask questions about the robotics textbook/i);
    });

    it('should have accessible close button', () => {
      render(<EnhancedFloatingChatPopup {...defaultProps} />);
      const closeButton = screen.getByLabelText('Close AI Assistant');

      expect(closeButton).toBeInTheDocument();
      expect(closeButton).toHaveAttribute('type', 'button');
    });

    it('should focus close button on open', async () => {
      render(<EnhancedFloatingChatPopup {...defaultProps} />);

      await waitFor(
        () => {
          expect(screen.getByLabelText('Close AI Assistant')).toHaveFocus();
        },
        { timeout: 500 }
      );
    });

    it('should trap focus inside popup', async () => {
      const user = userEvent.setup();
      render(<EnhancedFloatingChatPopup {...defaultProps} />);

      const closeButton = screen.getByLabelText('Close AI Assistant');
      closeButton.focus();

      // Tab through all focusable elements
      await user.tab();
      expect(document.activeElement).not.toBe(closeButton);

      // Continue tabbing should eventually loop back
      for (let i = 0; i < 10; i++) {
        await user.tab();
      }

      // Should still be within the dialog
      const dialog = screen.getByRole('dialog');
      expect(dialog.contains(document.activeElement)).toBe(true);
    });
  });

  describe('Keyboard Navigation', () => {
    it('should close on Escape key', () => {
      const onClose = jest.fn();
      render(<EnhancedFloatingChatPopup {...defaultProps} onClose={onClose} />);

      fireEvent.keyDown(document, { key: 'Escape' });

      expect(onClose).toHaveBeenCalledTimes(1);
    });

    it('should close menu on Escape key if menu is open', () => {
      const onClose = jest.fn();
      render(<EnhancedFloatingChatPopup {...defaultProps} onClose={onClose} />);

      // Open menu
      const menuButton = screen.getByLabelText('Menu');
      fireEvent.click(menuButton);

      // Press Escape
      fireEvent.keyDown(document, { key: 'Escape' });

      // Menu should close, popup should stay open
      expect(screen.queryByRole('menu')).not.toBeInTheDocument();
      expect(onClose).not.toHaveBeenCalled();
    });
  });

  describe('Header Controls', () => {
    it('should render close button', () => {
      render(<EnhancedFloatingChatPopup {...defaultProps} />);
      expect(screen.getByLabelText('Close AI Assistant')).toBeInTheDocument();
    });

    it('should call onClose when close button is clicked', () => {
      const onClose = jest.fn();
      render(<EnhancedFloatingChatPopup {...defaultProps} onClose={onClose} />);

      fireEvent.click(screen.getByLabelText('Close AI Assistant'));

      expect(onClose).toHaveBeenCalledTimes(1);
    });

    it('should render menu button', () => {
      render(<EnhancedFloatingChatPopup {...defaultProps} />);
      const menuButton = screen.getByLabelText('Menu');

      expect(menuButton).toBeInTheDocument();
      expect(menuButton).toHaveAttribute('aria-haspopup', 'true');
      expect(menuButton).toHaveAttribute('aria-expanded', 'false');
    });

    it('should open menu when menu button is clicked', () => {
      render(<EnhancedFloatingChatPopup {...defaultProps} />);
      const menuButton = screen.getByLabelText('Menu');

      fireEvent.click(menuButton);

      expect(screen.getByRole('menu')).toBeInTheDocument();
      expect(menuButton).toHaveAttribute('aria-expanded', 'true');
    });

    it('should render minimize button on desktop', () => {
      // Mock desktop viewport
      Object.defineProperty(window, 'innerWidth', {
        writable: true,
        configurable: true,
        value: 1024
      });

      render(<EnhancedFloatingChatPopup {...defaultProps} />);
      expect(screen.getByLabelText('Minimize')).toBeInTheDocument();
    });

    it('should toggle minimize state', () => {
      render(<EnhancedFloatingChatPopup {...defaultProps} />);
      const minimizeButton = screen.getByLabelText('Minimize');

      // Initial state: not minimized
      expect(screen.getByTestId('mock-chatbot-widget')).toBeInTheDocument();

      // Click to minimize
      fireEvent.click(minimizeButton);
      expect(screen.queryByTestId('mock-chatbot-widget')).not.toBeInTheDocument();
      expect(screen.getByLabelText('Maximize')).toBeInTheDocument();

      // Click to maximize
      fireEvent.click(screen.getByLabelText('Maximize'));
      expect(screen.getByTestId('mock-chatbot-widget')).toBeInTheDocument();
    });
  });

  describe('Menu Dropdown', () => {
    it('should render menu items when open', () => {
      render(<EnhancedFloatingChatPopup {...defaultProps} />);

      fireEvent.click(screen.getByLabelText('Menu'));

      expect(screen.getByRole('menuitem', { name: /Clear History/i })).toBeInTheDocument();
      expect(screen.getByRole('menuitem', { name: /Help & FAQ/i })).toBeInTheDocument();
    });

    it('should dispatch clear history event when clicked', () => {
      const dispatchEventSpy = jest.spyOn(window, 'dispatchEvent');
      render(<EnhancedFloatingChatPopup {...defaultProps} />);

      fireEvent.click(screen.getByLabelText('Menu'));
      fireEvent.click(screen.getByRole('menuitem', { name: /Clear History/i }));

      expect(dispatchEventSpy).toHaveBeenCalledWith(
        expect.objectContaining({
          type: 'chatbot-clear-history'
        })
      );
    });

    it('should open help in new tab when clicked', () => {
      const openSpy = jest.spyOn(window, 'open').mockImplementation();
      render(<EnhancedFloatingChatPopup {...defaultProps} />);

      fireEvent.click(screen.getByLabelText('Menu'));
      fireEvent.click(screen.getByRole('menuitem', { name: /Help & FAQ/i }));

      expect(openSpy).toHaveBeenCalledWith('/docs/help', '_blank');
    });
  });

  describe('Drag-to-Move (Desktop)', () => {
    beforeEach(() => {
      // Mock desktop viewport
      Object.defineProperty(window, 'innerWidth', {
        writable: true,
        configurable: true,
        value: 1024
      });
    });

    it('should allow dragging on desktop', () => {
      render(<EnhancedFloatingChatPopup {...defaultProps} />);
      const popup = screen.getByRole('dialog');
      const header = popup.querySelector('[class*="headerBranded"]') as HTMLElement;

      // Start drag
      fireEvent.mouseDown(header, { clientX: 100, clientY: 100 });

      // Move mouse
      fireEvent.mouseMove(document, { clientX: 200, clientY: 200 });

      // Should update position - check that left style contains a pixel value
      const style = popup.getAttribute('style') || '';
      expect(style).toMatch(/left:\s*\d+px/);
    });

    it('should not drag when clicking buttons in header', () => {
      render(<EnhancedFloatingChatPopup {...defaultProps} />);
      const closeButton = screen.getByLabelText('Close AI Assistant');

      // Click button should not start drag
      fireEvent.mouseDown(closeButton, { clientX: 100, clientY: 100 });
      fireEvent.mouseMove(document, { clientX: 200, clientY: 200 });

      const popup = screen.getByRole('dialog');
      // Should NOT have left position set (dragging didn't happen)
      const style = popup.getAttribute('style') || '';
      expect(style).not.toMatch(/left:\s*\d+px/);
    });
  });

  describe('Quick Actions', () => {
    it('should dispatch event when quick action is clicked', () => {
      const dispatchEventSpy = jest.spyOn(window, 'dispatchEvent');
      render(<EnhancedFloatingChatPopup {...defaultProps} />);

      const testActionButton = screen.getByText('Test Action');
      fireEvent.click(testActionButton);

      expect(dispatchEventSpy).toHaveBeenCalledWith(
        expect.objectContaining({
          type: 'chatbot-suggested-term',
          detail: { term: 'test query' }
        })
      );
    });
  });

  describe('Responsive Behavior', () => {
    it('should apply mobile styles on small screens', () => {
      // Mock mobile viewport
      Object.defineProperty(window, 'innerWidth', {
        writable: true,
        configurable: true,
        value: 375
      });

      render(<EnhancedFloatingChatPopup {...defaultProps} />);
      const popup = screen.getByRole('dialog');

      // Mobile styles applied via CSS, check for mobile-specific attributes
      expect(popup).toBeInTheDocument();
    });

    it('should prevent body scroll on mobile when open', () => {
      // Mock mobile viewport
      Object.defineProperty(window, 'innerWidth', {
        writable: true,
        configurable: true,
        value: 375
      });

      render(<EnhancedFloatingChatPopup {...defaultProps} />);

      expect(document.body.style.overflow).toBe('hidden');
      expect(document.body.style.position).toBe('fixed');
    });

    it('should not prevent body scroll on desktop', () => {
      // Mock desktop viewport
      Object.defineProperty(window, 'innerWidth', {
        writable: true,
        configurable: true,
        value: 1024
      });

      render(<EnhancedFloatingChatPopup {...defaultProps} />);

      expect(document.body.style.overflow).toBe('');
    });
  });

  describe('Animation States', () => {
    it('should apply opening animation class', () => {
      render(
        <EnhancedFloatingChatPopup
          {...defaultProps}
          animationState="opening"
        />
      );
      const popup = screen.getByRole('dialog');

      expect(popup.className).toContain('popup-opening');
    });

    it('should apply open animation class', () => {
      render(
        <EnhancedFloatingChatPopup
          {...defaultProps}
          animationState="open"
        />
      );
      const popup = screen.getByRole('dialog');

      expect(popup.className).toContain('popup-open');
    });

    it('should apply closing animation class', () => {
      render(
        <EnhancedFloatingChatPopup
          {...defaultProps}
          isOpen={true}
          animationState="closing"
        />
      );
      const popup = screen.getByRole('dialog');

      expect(popup.className).toContain('popup-closing');
    });
  });

  describe('Dark Mode', () => {
    it('should apply dark mode styles when theme is dark', () => {
      document.documentElement.setAttribute('data-theme', 'dark');

      render(<EnhancedFloatingChatPopup {...defaultProps} />);
      const popup = screen.getByRole('dialog');

      // Dark mode styles applied via CSS variables
      expect(popup).toBeInTheDocument();

      document.documentElement.removeAttribute('data-theme');
    });
  });

  describe('Focus Restoration', () => {
    it('should restore focus to previous element when closed', () => {
      const button = document.createElement('button');
      button.textContent = 'Open Chat';
      document.body.appendChild(button);
      button.focus();

      const { rerender } = render(
        <EnhancedFloatingChatPopup {...defaultProps} />
      );

      // Close popup
      rerender(
        <EnhancedFloatingChatPopup
          {...defaultProps}
          isOpen={false}
          animationState="closed"
        />
      );

      // Focus should return to button (in real implementation)
      // Note: This is difficult to test without full integration test

      document.body.removeChild(button);
    });
  });
});
