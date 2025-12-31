/**
 * LanguageToggle Component Tests
 * Phase 5: Translation Feature - T017
 */

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { LanguageToggle } from './index';
import { LanguageProvider, LanguageContext } from '../../context/LanguageContext';
import type { LanguageCode } from '../../types/translation';

// Mock localStorage
const localStorageMock = (() => {
  let store: Record<string, string> = {};
  return {
    getItem: jest.fn((key: string) => store[key] || null),
    setItem: jest.fn((key: string, value: string) => {
      store[key] = value;
    }),
    removeItem: jest.fn((key: string) => {
      delete store[key];
    }),
    clear: jest.fn(() => {
      store = {};
    }),
  };
})();

Object.defineProperty(window, 'localStorage', { value: localStorageMock });

// Custom render with provider
const renderWithProvider = (ui: React.ReactElement, initialLanguage: LanguageCode = 'en') => {
  const TestWrapper: React.FC<{ children: React.ReactNode }> = ({ children }) => {
    const [language, setLanguageState] = React.useState<LanguageCode>(initialLanguage);

    const setLanguage = (lang: LanguageCode) => {
      setLanguageState(lang);
    };

    return (
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
          setLanguage,
          hasTranslationError: false,
          setTranslationError: () => {},
          isLoading: false,
        }}
      >
        {children}
      </LanguageContext.Provider>
    );
  };

  return render(ui, { wrapper: TestWrapper });
};

describe('LanguageToggle', () => {
  beforeEach(() => {
    localStorageMock.clear();
    jest.clearAllMocks();
  });

  describe('Rendering', () => {
    it('renders without crashing', () => {
      renderWithProvider(<LanguageToggle />);
      expect(screen.getByRole('button')).toBeInTheDocument();
    });

    it('displays current language label', () => {
      renderWithProvider(<LanguageToggle />);
      expect(screen.getByText(/English|EN/i)).toBeInTheDocument();
    });

    it('displays Urdu label when language is Urdu', () => {
      renderWithProvider(<LanguageToggle />, 'ur');
      expect(screen.getByText(/اردو|UR/i)).toBeInTheDocument();
    });

    it('has proper button styling', () => {
      renderWithProvider(<LanguageToggle />);
      const button = screen.getByRole('button');
      expect(button).toHaveClass('languageToggle');
    });
  });

  describe('Accessibility', () => {
    it('has accessible name', () => {
      renderWithProvider(<LanguageToggle />);
      const button = screen.getByRole('button');
      expect(button).toHaveAccessibleName();
    });

    it('has aria-label describing the action', () => {
      renderWithProvider(<LanguageToggle />);
      const button = screen.getByRole('button');
      expect(button).toHaveAttribute('aria-label');
    });

    it('has aria-expanded when menu is open', async () => {
      const user = userEvent.setup();
      renderWithProvider(<LanguageToggle />);

      const button = screen.getByRole('button');
      expect(button).toHaveAttribute('aria-expanded', 'false');

      await user.click(button);
      expect(button).toHaveAttribute('aria-expanded', 'true');
    });

    it('has aria-haspopup for dropdown menu', () => {
      renderWithProvider(<LanguageToggle />);
      const button = screen.getByRole('button');
      expect(button).toHaveAttribute('aria-haspopup', 'menu');
    });

    it('meets 44x44px minimum touch target size', () => {
      renderWithProvider(<LanguageToggle />);
      const button = screen.getByRole('button');
      const styles = window.getComputedStyle(button);

      // Check min dimensions (allowing for 44px or larger)
      expect(parseInt(styles.minWidth) || parseInt(styles.width)).toBeGreaterThanOrEqual(44);
      expect(parseInt(styles.minHeight) || parseInt(styles.height)).toBeGreaterThanOrEqual(44);
    });

    it('is keyboard accessible', async () => {
      const user = userEvent.setup();
      renderWithProvider(<LanguageToggle />);

      const button = screen.getByRole('button');

      // Tab to focus
      await user.tab();
      expect(button).toHaveFocus();

      // Enter to open menu
      await user.keyboard('{Enter}');
      expect(button).toHaveAttribute('aria-expanded', 'true');
    });

    it('closes menu on Escape key', async () => {
      const user = userEvent.setup();
      renderWithProvider(<LanguageToggle />);

      const button = screen.getByRole('button');
      await user.click(button);
      expect(button).toHaveAttribute('aria-expanded', 'true');

      await user.keyboard('{Escape}');
      expect(button).toHaveAttribute('aria-expanded', 'false');
    });
  });

  describe('Interaction', () => {
    it('opens dropdown menu on click', async () => {
      const user = userEvent.setup();
      renderWithProvider(<LanguageToggle />);

      const button = screen.getByRole('button');
      await user.click(button);

      expect(screen.getByRole('menu')).toBeInTheDocument();
    });

    it('shows both language options in menu', async () => {
      const user = userEvent.setup();
      renderWithProvider(<LanguageToggle />);

      const button = screen.getByRole('button');
      await user.click(button);

      expect(screen.getByText('English')).toBeInTheDocument();
      expect(screen.getByText('اردو')).toBeInTheDocument();
    });

    it('switches to Urdu when Urdu option is selected', async () => {
      const user = userEvent.setup();
      renderWithProvider(<LanguageToggle />);

      const button = screen.getByRole('button');
      await user.click(button);

      const urduOption = screen.getByText('اردو');
      await user.click(urduOption);

      await waitFor(() => {
        expect(screen.getByText(/اردو|UR/i)).toBeInTheDocument();
      });
    });

    it('closes menu after selection', async () => {
      const user = userEvent.setup();
      renderWithProvider(<LanguageToggle />);

      const button = screen.getByRole('button');
      await user.click(button);

      const urduOption = screen.getByText('اردو');
      await user.click(urduOption);

      await waitFor(() => {
        expect(button).toHaveAttribute('aria-expanded', 'false');
      });
    });

    it('closes menu when clicking outside', async () => {
      const user = userEvent.setup();
      renderWithProvider(
        <div>
          <LanguageToggle />
          <div data-testid="outside">Outside</div>
        </div>
      );

      const button = screen.getByRole('button');
      await user.click(button);
      expect(button).toHaveAttribute('aria-expanded', 'true');

      await user.click(screen.getByTestId('outside'));
      await waitFor(() => {
        expect(button).toHaveAttribute('aria-expanded', 'false');
      });
    });
  });

  describe('Visual Indicators', () => {
    it('shows checkmark or highlight for current language', async () => {
      const user = userEvent.setup();
      renderWithProvider(<LanguageToggle />);

      const button = screen.getByRole('button');
      await user.click(button);

      const englishOption = screen.getByRole('menuitem', { name: /English/i });
      expect(englishOption).toHaveAttribute('aria-checked', 'true');
    });

    it('shows language flag or icon', () => {
      renderWithProvider(<LanguageToggle />);
      const button = screen.getByRole('button');

      // Should have an icon (globe or language icon)
      const icon = button.querySelector('svg, img, [data-icon]');
      expect(icon).toBeInTheDocument();
    });
  });

  describe('RTL Support', () => {
    it('works correctly in RTL mode', async () => {
      const user = userEvent.setup();
      renderWithProvider(<LanguageToggle />, 'ur');

      const button = screen.getByRole('button');
      expect(button).toBeInTheDocument();

      await user.click(button);
      expect(screen.getByRole('menu')).toBeInTheDocument();
    });
  });

  describe('Edge Cases', () => {
    it('handles rapid clicks gracefully', async () => {
      const user = userEvent.setup();
      renderWithProvider(<LanguageToggle />);

      const button = screen.getByRole('button');

      // Rapid clicks
      await user.click(button);
      await user.click(button);
      await user.click(button);

      // Should still be in a valid state
      expect(button).toHaveAttribute('aria-expanded');
    });

    it('maintains state after multiple language switches', async () => {
      const user = userEvent.setup();
      renderWithProvider(<LanguageToggle />);

      const button = screen.getByRole('button');

      // Switch to Urdu
      await user.click(button);
      await user.click(screen.getByText('اردو'));

      // Switch back to English
      await user.click(button);
      await user.click(screen.getByText('English'));

      // Should be English
      await waitFor(() => {
        expect(screen.getByText(/English|EN/i)).toBeInTheDocument();
      });
    });
  });
});

describe('LanguageToggle with LanguageProvider', () => {
  it('integrates correctly with LanguageProvider', () => {
    render(
      <LanguageProvider>
        <LanguageToggle />
      </LanguageProvider>
    );

    expect(screen.getByRole('button')).toBeInTheDocument();
  });
});
