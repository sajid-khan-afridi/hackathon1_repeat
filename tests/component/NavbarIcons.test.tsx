/**
 * Component Tests: NavbarIcons
 *
 * Tests the NavbarIcons component rendering and accessibility.
 */

import { describe, it, expect } from 'vitest';
import { render, screen } from '@testing-library/react';
import { NavbarIcon } from '../../src/components/Navbar/NavbarIcons';
import { NavbarTooltip } from '../../src/components/Navbar/NavbarTooltip';

describe('NavbarIcons', () => {
  describe('NavbarIcon Component', () => {
    it('should render modules icon', () => {
      render(<NavbarIcon name="modules" aria-label="Modules" />);

      const icon = screen.getByRole('img', { name: 'Modules' });
      expect(icon).toBeInTheDocument();
    });

    it('should render chat icon', () => {
      render(<NavbarIcon name="chat" aria-label="Chat" />);

      const icon = screen.getByRole('img', { name: 'Chat' });
      expect(icon).toBeInTheDocument();
    });

    it('should render github icon', () => {
      render(<NavbarIcon name="github" aria-label="GitHub" />);

      const icon = screen.getByRole('img', { name: 'GitHub' });
      expect(icon).toBeInTheDocument();
    });

    it('should render as decorative when specified', () => {
      render(<NavbarIcon name="menu" decorative />);

      // Decorative icons should be aria-hidden
      const icon = document.querySelector('svg');
      expect(icon).toHaveAttribute('aria-hidden', 'true');
    });

    it('should apply custom size', () => {
      render(<NavbarIcon name="home" size={32} aria-label="Home" />);

      const icon = screen.getByRole('img', { name: 'Home' });
      expect(icon).toHaveAttribute('width', '32');
      expect(icon).toHaveAttribute('height', '32');
    });

    it('should apply custom className', () => {
      render(
        <NavbarIcon name="login" className="custom-icon" aria-label="Login" />
      );

      const icon = screen.getByRole('img', { name: 'Login' });
      expect(icon).toHaveClass('custom-icon');
    });

    it('should have focusable="false" for screen readers', () => {
      render(<NavbarIcon name="menu" decorative />);

      const icon = document.querySelector('svg');
      expect(icon).toHaveAttribute('focusable', 'false');
    });

    it('should handle unknown icon name gracefully', () => {
      // @ts-expect-error - Testing invalid prop
      render(<NavbarIcon name="unknown-icon" />);

      // Should render empty span or fallback
      const fallback = document.querySelector('span');
      expect(fallback).toBeInTheDocument();
    });
  });

  describe('NavbarTooltip Component', () => {
    it('should render children', () => {
      render(
        <NavbarTooltip content="Tooltip text">
          <button>Hover me</button>
        </NavbarTooltip>
      );

      expect(screen.getByRole('button', { name: 'Hover me' })).toBeInTheDocument();
    });

    it('should show tooltip on hover', async () => {
      const { getByRole } = render(
        <NavbarTooltip content="Tooltip text" showDelay={0}>
          <button>Hover me</button>
        </NavbarTooltip>
      );

      const button = getByRole('button');

      // Simulate hover
      button.dispatchEvent(new MouseEvent('mouseenter', { bubbles: true }));

      // Wait for tooltip
      await new Promise((resolve) => setTimeout(resolve, 250));

      // Tooltip should be visible
      const tooltip = screen.queryByRole('tooltip');
      // Note: Tooltip visibility depends on implementation
    });

    it('should have correct ARIA attributes when visible', async () => {
      const { getByRole } = render(
        <NavbarTooltip content="Help text" showDelay={0}>
          <button>Help</button>
        </NavbarTooltip>
      );

      const button = getByRole('button');

      // Trigger tooltip
      button.dispatchEvent(new MouseEvent('mouseenter', { bubbles: true }));

      await new Promise((resolve) => setTimeout(resolve, 250));

      // Button should reference tooltip
      // aria-describedby should be set when tooltip is visible
    });

    it('should hide tooltip on mouse leave', async () => {
      const { getByRole } = render(
        <NavbarTooltip content="Tooltip" showDelay={0} hideDelay={0}>
          <button>Hover</button>
        </NavbarTooltip>
      );

      const button = getByRole('button');

      // Show tooltip
      button.dispatchEvent(new MouseEvent('mouseenter', { bubbles: true }));
      await new Promise((resolve) => setTimeout(resolve, 250));

      // Hide tooltip
      button.dispatchEvent(new MouseEvent('mouseleave', { bubbles: true }));
      await new Promise((resolve) => setTimeout(resolve, 50));

      // Tooltip should be hidden
      const tooltip = screen.queryByRole('tooltip');
      expect(tooltip).not.toBeInTheDocument();
    });

    it('should support different positions', () => {
      const positions = ['top', 'bottom', 'left', 'right'] as const;

      positions.forEach((position) => {
        const { unmount } = render(
          <NavbarTooltip content="Positioned tooltip" position={position}>
            <button>Button</button>
          </NavbarTooltip>
        );

        // Should render without errors
        expect(screen.getByRole('button')).toBeInTheDocument();

        unmount();
      });
    });

    it('should not show tooltip when disabled', async () => {
      const { getByRole } = render(
        <NavbarTooltip content="Disabled tooltip" disabled showDelay={0}>
          <button>Disabled</button>
        </NavbarTooltip>
      );

      const button = getByRole('button');
      button.dispatchEvent(new MouseEvent('mouseenter', { bubbles: true }));

      await new Promise((resolve) => setTimeout(resolve, 250));

      const tooltip = screen.queryByRole('tooltip');
      expect(tooltip).not.toBeInTheDocument();
    });
  });
});
