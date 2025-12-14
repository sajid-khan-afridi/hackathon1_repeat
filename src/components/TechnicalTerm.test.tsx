import React from 'react';
import { render, screen } from '@testing-library/react';
import TechnicalTerm from './TechnicalTerm';

// Mock the context module
const mockUseLanguageContext = jest.fn();

jest.mock('@site/src/context/LanguageContext', () => ({
  useLanguageContext: () => mockUseLanguageContext(),
}));

// Mock the glossary data
jest.mock('@site/src/data/technical-terms.json', () => ({
  terms: [
    {
      term: 'ROS 2',
      category: 'platform',
      translationStatus: 'preserve',
      contextualExplanation: 'روبوٹ آپریٹنگ سسٹم 2',
    },
    {
      term: 'publisher',
      category: 'ros-concept',
      translationStatus: 'preserve',
      contextualExplanation: 'پبلشر - پیغامات بھیجنے والا نوڈ',
    },
    {
      term: 'subscriber',
      category: 'ros-concept',
      translationStatus: 'preserve',
      contextualExplanation: 'سبسکرائبر - پیغامات وصول کرنے والا نوڈ',
    },
  ],
}), { virtual: true });

describe('TechnicalTerm', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  describe('in English mode', () => {
    beforeEach(() => {
      mockUseLanguageContext.mockReturnValue({
        language: 'en',
        setLanguage: jest.fn(),
      });
    });

    it('renders the term text', () => {
      render(<TechnicalTerm term="ROS 2" />);

      expect(screen.getByText('ROS 2')).toBeInTheDocument();
    });

    it('renders with custom explanation in title', () => {
      render(<TechnicalTerm term="ROS 2" explanation="Robot Operating System 2" />);

      const termElement = screen.getByRole('term');
      expect(termElement).toHaveAttribute('title', 'Robot Operating System 2');
    });

    it('renders term without visible transliteration', () => {
      render(<TechnicalTerm term="publisher" />);

      const termElement = screen.getByRole('term');
      expect(termElement).toBeInTheDocument();
      // In English mode, transliteration is only in sr-only span, not visible
      const visibleText = termElement.textContent;
      expect(visibleText).toBe('publisher');
    });

    it('handles unknown terms gracefully', () => {
      render(<TechnicalTerm term="unknown_term" />);

      expect(screen.getByText('unknown_term')).toBeInTheDocument();
    });
  });

  describe('in Urdu mode', () => {
    beforeEach(() => {
      mockUseLanguageContext.mockReturnValue({
        language: 'ur',
        setLanguage: jest.fn(),
      });
    });

    it('renders term with Urdu contextual explanation', () => {
      render(<TechnicalTerm term="ROS 2" />);

      expect(screen.getByText('ROS 2')).toBeInTheDocument();
      expect(screen.getByText(/روبوٹ آپریٹنگ سسٹم 2/)).toBeInTheDocument();
    });

    it('renders publisher with Urdu explanation', () => {
      render(<TechnicalTerm term="publisher" />);

      expect(screen.getByText('publisher')).toBeInTheDocument();
      expect(screen.getByText(/پبلشر - پیغامات بھیجنے والا نوڈ/)).toBeInTheDocument();
    });

    it('falls back to English display for unknown terms', () => {
      render(<TechnicalTerm term="unknown_urdu_term" />);

      expect(screen.getByText('unknown_urdu_term')).toBeInTheDocument();
      // Should not have any Urdu definition since term is not in glossary
      expect(screen.queryByRole('definition')).not.toBeInTheDocument();
    });

    it('marks original term with lang="en"', () => {
      render(<TechnicalTerm term="ROS 2" />);

      const englishTerm = screen.getByText('ROS 2');
      expect(englishTerm).toHaveAttribute('lang', 'en');
    });

    it('marks Urdu explanation with lang="ur"', () => {
      render(<TechnicalTerm term="ROS 2" />);

      const urduExplanation = screen.getByRole('definition');
      expect(urduExplanation).toHaveAttribute('lang', 'ur');
    });
  });

  describe('accessibility', () => {
    beforeEach(() => {
      mockUseLanguageContext.mockReturnValue({
        language: 'en',
        setLanguage: jest.fn(),
      });
    });

    it('has role="term" attribute', () => {
      render(<TechnicalTerm term="ROS 2" />);

      const termElement = screen.getByRole('term');
      expect(termElement).toBeInTheDocument();
    });

    it('is keyboard focusable with tabIndex', () => {
      render(<TechnicalTerm term="ROS 2" />);

      const termElement = screen.getByRole('term');
      expect(termElement).toHaveAttribute('tabIndex', '0');
    });

    it('has aria-label combining term and explanation', () => {
      render(<TechnicalTerm term="ROS 2" explanation="Robot Operating System 2" />);

      const termElement = screen.getByRole('term');
      expect(termElement.getAttribute('aria-label')).toContain('ROS 2');
      expect(termElement.getAttribute('aria-label')).toContain('Robot Operating System 2');
    });

    it('has hidden definition for screen readers', () => {
      render(<TechnicalTerm term="publisher" explanation="Node that sends messages" />);

      const definition = screen.getByRole('definition');
      expect(definition).toHaveClass('sr-only');
      expect(definition).toHaveTextContent('Node that sends messages');
    });

    it('links term to definition via aria-describedby in Urdu mode', () => {
      mockUseLanguageContext.mockReturnValue({
        language: 'ur',
        setLanguage: jest.fn(),
      });

      render(<TechnicalTerm term="ROS 2" />);

      const termElement = screen.getByRole('term');
      const definitionId = termElement.getAttribute('aria-describedby');

      expect(definitionId).toBeTruthy();
      expect(screen.getByRole('definition')).toHaveAttribute('id', definitionId);
    });
  });

  describe('CSS classes', () => {
    beforeEach(() => {
      mockUseLanguageContext.mockReturnValue({
        language: 'en',
        setLanguage: jest.fn(),
      });
    });

    it('applies technical-term class', () => {
      render(<TechnicalTerm term="ROS 2" />);

      const termElement = screen.getByRole('term');
      expect(termElement).toHaveClass('technical-term');
    });

    it('applies term-original class in Urdu mode', () => {
      mockUseLanguageContext.mockReturnValue({
        language: 'ur',
        setLanguage: jest.fn(),
      });

      render(<TechnicalTerm term="ROS 2" />);

      const originalTerm = screen.getByText('ROS 2');
      expect(originalTerm).toHaveClass('term-original');
    });

    it('applies term-transliteration class for Urdu explanation', () => {
      mockUseLanguageContext.mockReturnValue({
        language: 'ur',
        setLanguage: jest.fn(),
      });

      render(<TechnicalTerm term="ROS 2" />);

      const transliteration = screen.getByRole('definition');
      expect(transliteration).toHaveClass('term-transliteration');
    });
  });

  describe('glossary lookup', () => {
    beforeEach(() => {
      mockUseLanguageContext.mockReturnValue({
        language: 'ur',
        setLanguage: jest.fn(),
      });
    });

    it('finds term in glossary by exact match', () => {
      render(<TechnicalTerm term="subscriber" />);

      expect(screen.getByText(/سبسکرائبر/)).toBeInTheDocument();
    });

    it('uses provided explanation over glossary when given', () => {
      mockUseLanguageContext.mockReturnValue({
        language: 'en',
        setLanguage: jest.fn(),
      });

      render(<TechnicalTerm term="ROS 2" explanation="Custom explanation" />);

      const termElement = screen.getByRole('term');
      expect(termElement).toHaveAttribute('title', 'Custom explanation');
    });
  });
});
