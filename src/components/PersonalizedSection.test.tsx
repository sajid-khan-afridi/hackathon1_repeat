import React from 'react';
import { render, screen } from '@testing-library/react';
import PersonalizedSection from './PersonalizedSection';

// Mock the context module
const mockUseUserContext = jest.fn();

jest.mock('@site/src/context/UserContext', () => ({
  useUserContext: () => mockUseUserContext(),
}));

// Mock the useAuth hook to avoid requiring AuthProvider - use @site/ alias
jest.mock('@site/src/hooks/useAuth', () => ({
  useAuth: () => ({
    isAuthenticated: false,
    isLoading: false,
    user: null,
    profile: null,
    login: jest.fn(),
    signup: jest.fn(),
    logout: jest.fn(),
    refreshAuth: jest.fn(),
    updateProfile: jest.fn(),
    isProfileComplete: false,
    userEmail: null,
    userId: null,
  }),
}));

describe('PersonalizedSection', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  describe('when user profile matches', () => {
    it('renders children when experience level matches', () => {
      mockUseUserContext.mockReturnValue({
        userProfile: {
          experienceLevel: 'beginner',
          rosFamiliarity: 'novice',
          hardwareAccess: false,
        },
      });

      render(
        <PersonalizedSection level="beginner">
          <p>Beginner content</p>
        </PersonalizedSection>
      );

      expect(screen.getByText('Beginner content')).toBeInTheDocument();
    });

    it('renders children when all conditions match', () => {
      mockUseUserContext.mockReturnValue({
        userProfile: {
          experienceLevel: 'intermediate',
          rosFamiliarity: 'intermediate',
          hardwareAccess: true,
        },
      });

      render(
        <PersonalizedSection
          level="intermediate"
          rosFamiliarity="intermediate"
          hardwareAccess={true}
        >
          <p>Intermediate with hardware content</p>
        </PersonalizedSection>
      );

      expect(screen.getByText('Intermediate with hardware content')).toBeInTheDocument();
    });
  });

  describe('when user profile does not match', () => {
    it('returns null when experience level does not match', () => {
      mockUseUserContext.mockReturnValue({
        userProfile: {
          experienceLevel: 'advanced',
          rosFamiliarity: 'expert',
          hardwareAccess: true,
        },
      });

      const { container } = render(
        <PersonalizedSection level="beginner">
          <p>Beginner content</p>
        </PersonalizedSection>
      );

      expect(container.firstChild).toBeNull();
    });

    it('returns null when ROS familiarity does not match', () => {
      mockUseUserContext.mockReturnValue({
        userProfile: {
          experienceLevel: 'beginner',
          rosFamiliarity: 'expert',
          hardwareAccess: false,
        },
      });

      const { container } = render(
        <PersonalizedSection level="beginner" rosFamiliarity="novice">
          <p>Content for novice ROS users</p>
        </PersonalizedSection>
      );

      expect(container.firstChild).toBeNull();
    });

    it('returns null when hardware access does not match', () => {
      mockUseUserContext.mockReturnValue({
        userProfile: {
          experienceLevel: 'beginner',
          rosFamiliarity: 'novice',
          hardwareAccess: false,
        },
      });

      const { container } = render(
        <PersonalizedSection level="beginner" hardwareAccess={true}>
          <p>Content for users with hardware</p>
        </PersonalizedSection>
      );

      expect(container.firstChild).toBeNull();
    });
  });

  describe('when no user profile is set (anonymous user)', () => {
    it('renders children for all users', () => {
      mockUseUserContext.mockReturnValue({
        userProfile: null,
      });

      render(
        <PersonalizedSection level="beginner">
          <p>Default content for everyone</p>
        </PersonalizedSection>
      );

      expect(screen.getByText('Default content for everyone')).toBeInTheDocument();
    });
  });

  describe('accessibility', () => {
    it('has correct role attribute', () => {
      mockUseUserContext.mockReturnValue({
        userProfile: {
          experienceLevel: 'beginner',
          rosFamiliarity: 'novice',
          hardwareAccess: false,
        },
      });

      render(
        <PersonalizedSection level="beginner">
          <p>Accessible content</p>
        </PersonalizedSection>
      );

      const section = screen.getByRole('region');
      expect(section).toBeInTheDocument();
    });

    it('has descriptive aria-label', () => {
      mockUseUserContext.mockReturnValue({
        userProfile: {
          experienceLevel: 'beginner',
          rosFamiliarity: 'novice',
          hardwareAccess: false,
        },
      });

      render(
        <PersonalizedSection level="beginner">
          <p>Accessible content</p>
        </PersonalizedSection>
      );

      const section = screen.getByRole('region');
      expect(section).toHaveAttribute('aria-label', 'Content for beginner level learners');
    });

    it('includes all conditions in aria-label', () => {
      mockUseUserContext.mockReturnValue({
        userProfile: {
          experienceLevel: 'intermediate',
          rosFamiliarity: 'intermediate',
          hardwareAccess: true,
        },
      });

      render(
        <PersonalizedSection
          level="intermediate"
          rosFamiliarity="intermediate"
          hardwareAccess={true}
        >
          <p>Complex content</p>
        </PersonalizedSection>
      );

      const section = screen.getByRole('region');
      expect(section.getAttribute('aria-label')).toContain('intermediate level learners');
      expect(section.getAttribute('aria-label')).toContain('intermediate ROS familiarity');
      expect(section.getAttribute('aria-label')).toContain('with hardware access');
    });

    it('has correct data attributes for personalization level', () => {
      mockUseUserContext.mockReturnValue({
        userProfile: {
          experienceLevel: 'advanced',
          rosFamiliarity: 'expert',
          hardwareAccess: true,
        },
      });

      render(
        <PersonalizedSection level="advanced">
          <p>Advanced content</p>
        </PersonalizedSection>
      );

      const section = screen.getByRole('region');
      expect(section).toHaveAttribute('data-personalization-level', 'advanced');
    });
  });

  describe('CSS classes', () => {
    it('applies level-specific CSS class', () => {
      mockUseUserContext.mockReturnValue({
        userProfile: {
          experienceLevel: 'beginner',
          rosFamiliarity: 'novice',
          hardwareAccess: false,
        },
      });

      render(
        <PersonalizedSection level="beginner">
          <p>Styled content</p>
        </PersonalizedSection>
      );

      const section = screen.getByRole('region');
      expect(section).toHaveClass('personalized-section');
      expect(section).toHaveClass('personalized-section--beginner');
    });

    it('applies level CSS class regardless of user profile', () => {
      mockUseUserContext.mockReturnValue({
        userProfile: null,
      });

      render(
        <PersonalizedSection level="intermediate">
          <p>Default styled content</p>
        </PersonalizedSection>
      );

      const section = screen.getByRole('region');
      expect(section).toHaveClass('personalized-section');
      // Class is based on level prop, not user profile
      expect(section).toHaveClass('personalized-section--intermediate');
    });
  });
});
