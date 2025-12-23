/**
 * Component Tests for PersonalizedSection
 * T060: Language filtering
 * T061: Experience level filtering
 * T062: Hardware filtering
 */

import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';
import PersonalizedSection from '../PersonalizedSection';
import { UserContext } from '../../context/UserContext';

// Mock user profiles
const mockBeginnerProfile = {
  experience_level: 'beginner',
  preferred_language: 'python',
  hardware_access: 'simulation_only',
};

const mockAdvancedProfile = {
  experience_level: 'advanced',
  preferred_language: 'cpp',
  hardware_access: 'physical_robot',
};

// T060: Language filtering test
describe('PersonalizedSection - Language Filtering', () => {
  test('shows Python content for Python preference', () => {
    const { container } = render(
      <UserContext.Provider value={{ profile: mockBeginnerProfile }}>
        <PersonalizedSection language="python">
          <div>Python content</div>
        </PersonalizedSection>
      </UserContext.Provider>
    );

    expect(container.textContent).toContain('Python content');
  });

  test('hides C++ content for Python preference', () => {
    const { container } = render(
      <UserContext.Provider value={{ profile: mockBeginnerProfile }}>
        <PersonalizedSection language="cpp">
          <div>C++ content</div>
        </PersonalizedSection>
      </UserContext.Provider>
    );

    expect(container.textContent).not.toContain('C++ content');
  });

  test('shows all content for "both" language preference', () => {
    const bothProfile = { ...mockBeginnerProfile, preferred_language: 'both' };

    const { container } = render(
      <UserContext.Provider value={{ profile: bothProfile }}>
        <PersonalizedSection language="python">
          <div>Python content</div>
        </PersonalizedSection>
      </UserContext.Provider>
    );

    expect(container.textContent).toContain('Python content');
  });
});

// T061: Experience level filtering test
describe('PersonalizedSection - Experience Level Filtering', () => {
  test('shows beginner content for beginner users', () => {
    const { container } = render(
      <UserContext.Provider value={{ profile: mockBeginnerProfile }}>
        <PersonalizedSection level="beginner">
          <div>Beginner content</div>
        </PersonalizedSection>
      </UserContext.Provider>
    );

    expect(container.textContent).toContain('Beginner content');
  });

  test('hides advanced content for beginner users', () => {
    const { container } = render(
      <UserContext.Provider value={{ profile: mockBeginnerProfile }}>
        <PersonalizedSection level="advanced">
          <div>Advanced content</div>
        </PersonalizedSection>
      </UserContext.Provider>
    );

    expect(container.textContent).not.toContain('Advanced content');
  });

  test('shows advanced content for advanced users', () => {
    const { container } = render(
      <UserContext.Provider value={{ profile: mockAdvancedProfile }}>
        <PersonalizedSection level="advanced">
          <div>Advanced content</div>
        </PersonalizedSection>
      </UserContext.Provider>
    );

    expect(container.textContent).toContain('Advanced content');
  });
});

// T062: Hardware filtering test
describe('PersonalizedSection - Hardware Filtering', () => {
  test('shows hardware content for users with physical robots', () => {
    const { container } = render(
      <UserContext.Provider value={{ profile: mockAdvancedProfile }}>
        <PersonalizedSection hardware="physical">
          <div>Hardware setup instructions</div>
        </PersonalizedSection>
      </UserContext.Provider>
    );

    expect(container.textContent).toContain('Hardware setup instructions');
  });

  test('hides hardware content for simulation-only users', () => {
    const { container } = render(
      <UserContext.Provider value={{ profile: mockBeginnerProfile }}>
        <PersonalizedSection hardware="physical">
          <div>Hardware setup instructions</div>
        </PersonalizedSection>
      </UserContext.Provider>
    );

    expect(container.textContent).not.toContain('Hardware setup instructions');
  });

  test('shows simulation content for all users', () => {
    const { container } = render(
      <UserContext.Provider value={{ profile: mockBeginnerProfile }}>
        <PersonalizedSection hardware="simulation">
          <div>Simulation instructions</div>
        </PersonalizedSection>
      </UserContext.Provider>
    );

    expect(container.textContent).toContain('Simulation instructions');
  });
});
