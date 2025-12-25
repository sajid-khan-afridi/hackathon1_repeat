/**
 * Component Tests for PersonalizedSection
 * T060: Language filtering
 * T061: Experience level filtering
 * T062: Hardware filtering
 */

import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';

// Define UserProfile type for tests
interface UserProfile {
  experienceLevel: 'beginner' | 'intermediate' | 'advanced';
  rosFamiliarity: 'novice' | 'intermediate' | 'expert';
  hardwareAccess: boolean;
  preferredLanguage?: 'python' | 'cpp' | 'both';
}

// Mock user profiles
const mockBeginnerProfile: UserProfile = {
  experienceLevel: 'beginner',
  rosFamiliarity: 'novice',
  hardwareAccess: false,
  preferredLanguage: 'python',
};

const mockAdvancedProfile: UserProfile = {
  experienceLevel: 'advanced',
  rosFamiliarity: 'expert',
  hardwareAccess: true,
  preferredLanguage: 'cpp',
};

// Track current mock profile
let currentMockProfile: UserProfile | null = null;

// Mock the useAuth hook - use @site/ alias that the component uses
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

// Mock the UserContext to return controlled values
jest.mock('@site/src/context/UserContext', () => ({
  useUserContext: () => ({
    userProfile: currentMockProfile,
    setUserProfile: jest.fn(),
  }),
}));

// Import after mocks
import PersonalizedSection from '../PersonalizedSection';

// Helper to set mock profile for a test
const setMockProfile = (profile: UserProfile | null) => {
  currentMockProfile = profile;
};

// T060: Language filtering test
describe('PersonalizedSection - Language Filtering', () => {
  beforeEach(() => {
    setMockProfile(null);
  });

  test('shows Python content for Python preference', () => {
    setMockProfile(mockBeginnerProfile);
    const { container } = render(
      <PersonalizedSection language="python">
        <div>Python content</div>
      </PersonalizedSection>
    );

    expect(container.textContent).toContain('Python content');
  });

  test('hides C++ content for Python preference', () => {
    setMockProfile(mockBeginnerProfile);
    const { container } = render(
      <PersonalizedSection language="cpp">
        <div>C++ content</div>
      </PersonalizedSection>
    );

    expect(container.textContent).not.toContain('C++ content');
  });

  test('shows all content for "both" language preference', () => {
    const bothProfile: UserProfile = { ...mockBeginnerProfile, preferredLanguage: 'both' };
    setMockProfile(bothProfile);
    const { container } = render(
      <PersonalizedSection language="python">
        <div>Python content</div>
      </PersonalizedSection>
    );

    expect(container.textContent).toContain('Python content');
  });
});

// T061: Experience level filtering test
describe('PersonalizedSection - Experience Level Filtering', () => {
  beforeEach(() => {
    setMockProfile(null);
  });

  test('shows beginner content for beginner users', () => {
    setMockProfile(mockBeginnerProfile);
    const { container } = render(
      <PersonalizedSection level="beginner">
        <div>Beginner content</div>
      </PersonalizedSection>
    );

    expect(container.textContent).toContain('Beginner content');
  });

  test('hides advanced content for beginner users', () => {
    setMockProfile(mockBeginnerProfile);
    const { container } = render(
      <PersonalizedSection level="advanced">
        <div>Advanced content</div>
      </PersonalizedSection>
    );

    expect(container.textContent).not.toContain('Advanced content');
  });

  test('shows advanced content for advanced users', () => {
    setMockProfile(mockAdvancedProfile);
    const { container } = render(
      <PersonalizedSection level="advanced">
        <div>Advanced content</div>
      </PersonalizedSection>
    );

    expect(container.textContent).toContain('Advanced content');
  });
});

// T062: Hardware filtering test
describe('PersonalizedSection - Hardware Filtering', () => {
  beforeEach(() => {
    setMockProfile(null);
  });

  test('shows hardware content for users with physical robots', () => {
    setMockProfile(mockAdvancedProfile);
    const { container } = render(
      <PersonalizedSection hardware="physical">
        <div>Hardware setup instructions</div>
      </PersonalizedSection>
    );

    expect(container.textContent).toContain('Hardware setup instructions');
  });

  test('hides hardware content for simulation-only users', () => {
    setMockProfile(mockBeginnerProfile);
    const { container } = render(
      <PersonalizedSection hardware="physical">
        <div>Hardware setup instructions</div>
      </PersonalizedSection>
    );

    expect(container.textContent).not.toContain('Hardware setup instructions');
  });

  test('shows simulation content for all users', () => {
    setMockProfile(mockBeginnerProfile);
    const { container } = render(
      <PersonalizedSection hardware="simulation">
        <div>Simulation instructions</div>
      </PersonalizedSection>
    );

    expect(container.textContent).toContain('Simulation instructions');
  });
});
