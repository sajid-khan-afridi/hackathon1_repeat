/**
 * Component Tests: SkeletonLoader
 *
 * Tests the SkeletonLoader component rendering and variants.
 */

import { describe, it, expect } from 'vitest';
import { render, screen } from '@testing-library/react';
import { SkeletonLoader } from '../../src/components/SkeletonLoader/SkeletonLoader';
import { TypingIndicator } from '../../src/components/SkeletonLoader/TypingIndicator';
import { ProgressBar } from '../../src/components/SkeletonLoader/ProgressBar';

describe('SkeletonLoader', () => {
  describe('SkeletonLoader Component', () => {
    it('should render with default props', () => {
      render(<SkeletonLoader />);

      const skeleton = document.querySelector('[class*="skeleton"]');
      expect(skeleton).toBeInTheDocument();
    });

    it('should render with custom width and height', () => {
      render(<SkeletonLoader width={200} height={40} />);

      const skeleton = document.querySelector('[class*="skeleton"]');
      expect(skeleton).toHaveStyle({ width: '200px', height: '40px' });
    });

    it('should render text variant', () => {
      render(<SkeletonLoader variant="text" />);

      const skeleton = document.querySelector('[class*="skeleton"]');
      expect(skeleton).toBeInTheDocument();
    });

    it('should render circular variant', () => {
      render(<SkeletonLoader variant="circular" />);

      const skeleton = document.querySelector('[class*="skeleton"]');
      expect(skeleton).toHaveClass(/circular/i);
    });

    it('should render rectangular variant', () => {
      render(<SkeletonLoader variant="rectangular" />);

      const skeleton = document.querySelector('[class*="skeleton"]');
      expect(skeleton).toHaveClass(/rectangular/i);
    });

    it('should apply custom className', () => {
      render(<SkeletonLoader className="custom-skeleton" />);

      const skeleton = document.querySelector('[class*="skeleton"]');
      expect(skeleton).toHaveClass('custom-skeleton');
    });

    it('should have shimmer animation class', () => {
      render(<SkeletonLoader />);

      const skeleton = document.querySelector('[class*="skeleton"]');
      // Should have animation or shimmer class
      expect(skeleton?.className).toMatch(/shimmer|animate/i);
    });
  });

  describe('TypingIndicator Component', () => {
    it('should render with default props', () => {
      render(<TypingIndicator />);

      const indicator = screen.getByRole('status');
      expect(indicator).toBeInTheDocument();
    });

    it('should render three dots', () => {
      render(<TypingIndicator />);

      const dots = document.querySelectorAll('[class*="dot"]');
      expect(dots.length).toBe(3);
    });

    it('should have accessible label', () => {
      render(<TypingIndicator label="AI is typing" />);

      expect(screen.getByText('AI is typing')).toBeInTheDocument();
    });

    it('should render small size variant', () => {
      render(<TypingIndicator size="small" />);

      const indicator = screen.getByRole('status');
      expect(indicator).toHaveClass(/small/i);
    });

    it('should render large size variant', () => {
      render(<TypingIndicator size="large" />);

      const indicator = screen.getByRole('status');
      expect(indicator).toHaveClass(/large/i);
    });

    it('should apply custom className', () => {
      render(<TypingIndicator className="custom-typing" />);

      const indicator = screen.getByRole('status');
      expect(indicator).toHaveClass('custom-typing');
    });
  });

  describe('ProgressBar Component', () => {
    it('should render with value', () => {
      render(<ProgressBar value={50} />);

      const progressbar = screen.getByRole('progressbar');
      expect(progressbar).toBeInTheDocument();
    });

    it('should have correct ARIA attributes', () => {
      render(<ProgressBar value={75} />);

      const progressbar = screen.getByRole('progressbar');
      expect(progressbar).toHaveAttribute('aria-valuenow', '75');
      expect(progressbar).toHaveAttribute('aria-valuemin', '0');
      expect(progressbar).toHaveAttribute('aria-valuemax', '100');
    });

    it('should clamp value to 0-100', () => {
      render(<ProgressBar value={150} />);

      const progressbar = screen.getByRole('progressbar');
      expect(progressbar).toHaveAttribute('aria-valuenow', '100');
    });

    it('should handle negative value', () => {
      render(<ProgressBar value={-10} />);

      const progressbar = screen.getByRole('progressbar');
      expect(progressbar).toHaveAttribute('aria-valuenow', '0');
    });

    it('should render indeterminate variant', () => {
      render(<ProgressBar indeterminate />);

      const progressbar = screen.getByRole('progressbar');
      expect(progressbar).not.toHaveAttribute('aria-valuenow');
    });

    it('should apply custom className', () => {
      render(<ProgressBar value={50} className="custom-progress" />);

      const progressbar = screen.getByRole('progressbar');
      expect(progressbar).toHaveClass('custom-progress');
    });
  });
});
