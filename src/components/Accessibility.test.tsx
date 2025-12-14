import React from 'react';
import { render } from '@testing-library/react';
import { axe, toHaveNoViolations } from 'jest-axe';
import ThemeToggle from './ThemeToggle';
import PersonalizedSection from './PersonalizedSection';
import TechnicalTerm from './TechnicalTerm';
import { UserProvider } from '../context/UserContext';
import { LanguageProvider } from '../context/LanguageContext';

// Extend Jest matchers
expect.extend(toHaveNoViolations);

describe('Accessibility Tests', () => {
  it('ThemeToggle should not have accessibility violations', async () => {
    const { container } = render(<ThemeToggle />);
    const results = await axe(container);
    expect(results).toHaveNoViolations();
  });

  it('PersonalizedSection should not have accessibility violations', async () => {
    const { container } = render(
      <UserProvider>
        <PersonalizedSection level="beginner" hardwareAccess={false}>
          <p>Test content for beginners</p>
        </PersonalizedSection>
      </UserProvider>
    );
    const results = await axe(container);
    expect(results).toHaveNoViolations();
  });

  it('TechnicalTerm should not have accessibility violations', async () => {
    const { container } = render(
      <LanguageProvider>
        <TechnicalTerm term="ROS" explanation="Robot Operating System" />
      </LanguageProvider>
    );
    const results = await axe(container);
    expect(results).toHaveNoViolations();
  });
});
