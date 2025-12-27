/**
 * GlossaryTooltip Component Tests
 * Phase 5: Translation Feature - T034
 */

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { GlossaryTooltip } from './index';
import { LanguageContext } from '../../context/LanguageContext';
import type { LanguageCode } from '../../types/translation';

// Mock the useGlossary hook
jest.mock('../../hooks/useGlossary', () => ({
  useGlossary: () => ({
    getTermById: (id: string) => {
      const terms: Record<string, any> = {
        'ros-2': {
          id: 'ros-2',
          english: 'ROS 2',
          urduTransliteration: 'آر او ایس ٹو',
          definition: 'Robot Operating System version 2, a middleware for robotics.',
          definitionUrdu: 'روبوٹ آپریٹنگ سسٹم ورژن 2، روبوٹکس کے لیے مڈل ویئر۔',
          category: 'Middleware',
          relatedTerms: ['urdf', 'gazebo'],
        },
        'urdf': {
          id: 'urdf',
          english: 'URDF',
          urduTransliteration: 'یو آر ڈی ایف',
          definition: 'Unified Robot Description Format for robot models.',
          definitionUrdu: 'روبوٹ ماڈلز کے لیے یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ۔',
          category: 'Standards',
          relatedTerms: [],
        },
        'gazebo': {
          id: 'gazebo',
          english: 'Gazebo',
          urduTransliteration: 'گزیبو',
          definition: 'A 3D robot simulator.',
          definitionUrdu: 'ایک 3D روبوٹ سمیولیٹر۔',
          category: 'Simulation',
          relatedTerms: [],
        },
      };
      return terms[id];
    },
    getRelatedTerms: (termId: string) => {
      if (termId === 'ros-2') {
        return [
          {
            id: 'urdf',
            english: 'URDF',
            urduTransliteration: 'یو آر ڈی ایف',
            definition: 'Unified Robot Description Format.',
            definitionUrdu: 'یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ۔',
            category: 'Standards',
          },
          {
            id: 'gazebo',
            english: 'Gazebo',
            urduTransliteration: 'گزیبو',
            definition: 'A 3D robot simulator.',
            definitionUrdu: 'ایک 3D روبوٹ سمیولیٹر۔',
            category: 'Simulation',
          },
        ];
      }
      return [];
    },
    terms: [],
    filteredTerms: [],
    searchQuery: '',
    setSearchQuery: jest.fn(),
    categoryFilter: null,
    setCategoryFilter: jest.fn(),
    categories: [],
    hasTerm: (id: string) => ['ros-2', 'urdf', 'gazebo'].includes(id),
    isLoading: false,
    error: null,
  }),
}));

// Custom render with LanguageContext
const renderWithLanguage = (
  ui: React.ReactElement,
  language: LanguageCode = 'en'
) => {
  return render(
    <LanguageContext.Provider
      value={{
        language,
        direction: language === 'ur' ? 'rtl' : 'ltr',
        config: {
          code: language,
          label: language === 'ur' ? 'اردو' : 'English',
          direction: language === 'ur' ? 'rtl' : 'ltr',
          htmlLang: language === 'ur' ? 'ur-PK' : 'en-US',
        },
        setLanguage: jest.fn(),
        hasTranslationError: false,
        setTranslationError: jest.fn(),
        isLoading: false,
      }}
    >
      {ui}
    </LanguageContext.Provider>
  );
};

describe('GlossaryTooltip', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  describe('Rendering', () => {
    it('renders children text', () => {
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>
      );
      expect(screen.getByText('ROS 2')).toBeInTheDocument();
    });

    it('renders as inline span', () => {
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>
      );
      const container = screen.getByText('ROS 2').parentElement;
      expect(container?.tagName.toLowerCase()).toBe('span');
    });

    it('applies custom className', () => {
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2" className="custom-class">
          ROS 2
        </GlossaryTooltip>
      );
      const container = screen.getByText('ROS 2').closest('.container');
      expect(container).toHaveClass('custom-class');
    });

    it('renders plain text for unknown term', () => {
      renderWithLanguage(
        <GlossaryTooltip termId="unknown-term">Unknown</GlossaryTooltip>
      );
      expect(screen.getByText('Unknown')).toBeInTheDocument();
      // Should not have technical term styling
      expect(screen.getByText('Unknown')).not.toHaveAttribute('data-technical-term');
    });
  });

  describe('Tooltip Display', () => {
    it('shows tooltip on hover', async () => {
      const user = userEvent.setup();
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>
      );

      const term = screen.getByText('ROS 2');
      await user.hover(term.parentElement!);

      await waitFor(() => {
        expect(screen.getByRole('tooltip')).toBeInTheDocument();
      });
    });

    it('hides tooltip on mouse leave', async () => {
      const user = userEvent.setup();
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>
      );

      const container = screen.getByText('ROS 2').closest('.container');
      await user.hover(container!);

      await waitFor(() => {
        expect(screen.getByRole('tooltip')).toBeInTheDocument();
      });

      await user.unhover(container!);

      await waitFor(() => {
        expect(screen.queryByRole('tooltip')).not.toBeInTheDocument();
      }, { timeout: 500 });
    });

    it('shows English definition in English mode', async () => {
      const user = userEvent.setup();
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>,
        'en'
      );

      const container = screen.getByText('ROS 2').closest('.container');
      await user.hover(container!);

      await waitFor(() => {
        expect(screen.getByText(/Robot Operating System version 2/)).toBeInTheDocument();
      });
    });

    it('shows Urdu definition in Urdu mode', async () => {
      const user = userEvent.setup();
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>,
        'ur'
      );

      const container = screen.getByText('ROS 2').closest('.container');
      await user.hover(container!);

      await waitFor(() => {
        expect(screen.getByText(/روبوٹ آپریٹنگ سسٹم/)).toBeInTheDocument();
      });
    });

    it('shows related terms in tooltip', async () => {
      const user = userEvent.setup();
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>
      );

      const container = screen.getByText('ROS 2').closest('.container');
      await user.hover(container!);

      await waitFor(() => {
        expect(screen.getByText('URDF')).toBeInTheDocument();
        expect(screen.getByText('Gazebo')).toBeInTheDocument();
      });
    });

    it('shows category badge in tooltip', async () => {
      const user = userEvent.setup();
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>
      );

      const container = screen.getByText('ROS 2').closest('.container');
      await user.hover(container!);

      await waitFor(() => {
        expect(screen.getByText('Middleware')).toBeInTheDocument();
      });
    });
  });

  describe('Accessibility', () => {
    it('has tabIndex for keyboard access', () => {
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>
      );
      const term = screen.getByText('ROS 2');
      expect(term).toHaveAttribute('tabIndex', '0');
    });

    it('has button role', () => {
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>
      );
      const term = screen.getByRole('button');
      expect(term).toBeInTheDocument();
    });

    it('shows tooltip on focus', async () => {
      const user = userEvent.setup();
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>
      );

      const term = screen.getByText('ROS 2');
      fireEvent.focus(term.parentElement!);

      await waitFor(() => {
        expect(screen.getByRole('tooltip')).toBeInTheDocument();
      });
    });

    it('hides tooltip on blur', async () => {
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>
      );

      const container = screen.getByText('ROS 2').closest('.container');
      fireEvent.focus(container!);

      await waitFor(() => {
        expect(screen.getByRole('tooltip')).toBeInTheDocument();
      });

      fireEvent.blur(container!);

      await waitFor(() => {
        expect(screen.queryByRole('tooltip')).not.toBeInTheDocument();
      }, { timeout: 500 });
    });

    it('closes tooltip on Escape key', async () => {
      const user = userEvent.setup();
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>
      );

      const container = screen.getByText('ROS 2').closest('.container');
      fireEvent.focus(container!);

      await waitFor(() => {
        expect(screen.getByRole('tooltip')).toBeInTheDocument();
      });

      fireEvent.keyDown(container!, { key: 'Escape' });

      await waitFor(() => {
        expect(screen.queryByRole('tooltip')).not.toBeInTheDocument();
      });
    });

    it('has aria-describedby when tooltip is visible', async () => {
      const user = userEvent.setup();
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>
      );

      const container = screen.getByText('ROS 2').closest('.container');
      await user.hover(container!);

      await waitFor(() => {
        const term = screen.getByRole('button');
        expect(term).toHaveAttribute('aria-describedby', 'tooltip-ros-2');
      });
    });

    it('tooltip has id matching aria-describedby', async () => {
      const user = userEvent.setup();
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>
      );

      const container = screen.getByText('ROS 2').closest('.container');
      await user.hover(container!);

      await waitFor(() => {
        const tooltip = screen.getByRole('tooltip');
        expect(tooltip).toHaveAttribute('id', 'tooltip-ros-2');
      });
    });
  });

  describe('Styling', () => {
    it('has visual distinction for technical terms', () => {
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>
      );
      const term = screen.getByText('ROS 2');
      expect(term).toHaveAttribute('data-technical-term');
    });

    it('has cursor help style', () => {
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>
      );
      const term = screen.getByText('ROS 2');
      expect(term).toHaveClass('term');
    });
  });

  describe('RTL Support', () => {
    it('works in RTL mode', async () => {
      const user = userEvent.setup();
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>,
        'ur'
      );

      const container = screen.getByText('ROS 2').closest('.container');
      await user.hover(container!);

      await waitFor(() => {
        expect(screen.getByRole('tooltip')).toBeInTheDocument();
      });
    });

    it('shows Urdu transliteration in Urdu mode', async () => {
      const user = userEvent.setup();
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>,
        'ur'
      );

      const container = screen.getByText('ROS 2').closest('.container');
      await user.hover(container!);

      await waitFor(() => {
        expect(screen.getByText('آر او ایس ٹو')).toBeInTheDocument();
      });
    });
  });

  describe('Edge Cases', () => {
    it('handles undefined term gracefully', () => {
      renderWithLanguage(
        <GlossaryTooltip termId="nonexistent">Text</GlossaryTooltip>
      );
      expect(screen.getByText('Text')).toBeInTheDocument();
    });

    it('handles empty children', () => {
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">{''}</GlossaryTooltip>
      );
      // Should not throw
    });

    it('handles rapid hover in/out', async () => {
      const user = userEvent.setup();
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>
      );

      const container = screen.getByText('ROS 2').closest('.container');

      // Rapid hover in/out
      await user.hover(container!);
      await user.unhover(container!);
      await user.hover(container!);

      // Should end up visible
      await waitFor(() => {
        expect(screen.getByRole('tooltip')).toBeInTheDocument();
      });
    });

    it('allows moving mouse to tooltip', async () => {
      const user = userEvent.setup();
      renderWithLanguage(
        <GlossaryTooltip termId="ros-2">ROS 2</GlossaryTooltip>
      );

      const container = screen.getByText('ROS 2').closest('.container');
      await user.hover(container!);

      await waitFor(() => {
        expect(screen.getByRole('tooltip')).toBeInTheDocument();
      });

      // Move to tooltip (it should stay visible)
      const tooltip = screen.getByRole('tooltip');
      await user.hover(tooltip);

      expect(tooltip).toBeInTheDocument();
    });
  });
});
