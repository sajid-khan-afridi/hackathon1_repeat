import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import { DocusaurusContextProvider } from '@docusaurus/theme-common/internal';
import ThemeToggle from './index';

const mockSetLightTheme = jest.fn();
const mockSetDarkTheme = jest.fn();

describe('ThemeToggle', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('renders null on server side', () => {
    const { container } = render(
      <DocusaurusContextProvider>
        <ThemeToggle />
      </DocusaurusContextProvider>,
      { wrapper: ({ children }) => <div>{children}</div> }
    );
    expect(container.firstChild).toBeNull();
  });

  it('shows sun icon in dark mode', () => {
    render(
      <DocusaurusContextProvider
        value={{
          isClient: true,
          siteConfig: {},
          i18n: { currentLocale: 'en' },
          colorMode: {
              colorMode: 'dark' as const,
              setColorMode: mockSetLightTheme,
            }
        }}
      >
        <ThemeToggle />
      </DocusaurusContextProvider>
    );

    const toggle = screen.getByRole('button', { name: /Switch to light mode/i });
    expect(toggle).toBeInTheDocument();

    // Check for sun icon (should be present in dark mode)
    const icon = toggle.querySelector('svg');
    expect(icon).toBeInTheDocument();
  });

  it('shows moon icon in light mode', () => {
    render(
      <DocusaurusContextProvider
        value={{
          isClient: true,
          siteConfig: {},
          i18n: { currentLocale: 'en' },
          colorMode: {
              colorMode: 'light' as const,
              setColorMode: mockSetDarkTheme,
            }
        }}
      >
        <ThemeToggle />
      </DocusaurusContextProvider>
    );

    const toggle = screen.getByRole('button', { name: /Switch to dark mode/i });
    expect(toggle).toBeInTheDocument();

    // Check for moon icon (should be present in light mode)
    const icon = toggle.querySelector('svg');
    expect(icon).toBeInTheDocument();
  });

  it('calls setColorMode when clicked in light mode', () => {
    render(
      <DocusaurusContextProvider
        value={{
          isClient: true,
          siteConfig: {},
          i18n: { currentLocale: 'en' },
          colorMode: {
              colorMode: 'light' as const,
              setColorMode: mockSetDarkTheme,
            }
        }}
      >
        <ThemeToggle />
      </DocusaurusContextProvider>
    );

    const toggle = screen.getByRole('button');
    fireEvent.click(toggle);

    expect(mockSetDarkTheme).toHaveBeenCalledWith('dark');
  });

  it('calls setColorMode when clicked in dark mode', () => {
    render(
      <DocusaurusContextProvider
        value={{
          isClient: true,
          siteConfig: {},
          i18n: { currentLocale: 'en' },
          colorMode: {
              colorMode: 'dark' as const,
              setColorMode: mockSetLightTheme,
            }
        }}
      >
        <ThemeToggle />
      </DocusaurusContextProvider>
    );

    const toggle = screen.getByRole('button');
    fireEvent.click(toggle);

    expect(mockSetLightTheme).toHaveBeenCalledWith('light');
  });

  it('has proper accessibility attributes', () => {
    render(
      <DocusaurusContextProvider
        value={{
          isClient: true,
          siteConfig: {},
          i18n: { currentLocale: 'en' },
          colorMode: {
              colorMode: 'light' as const,
              setColorMode: mockSetDarkTheme,
            }
        }}
      >
        <ThemeToggle />
      </DocusaurusContextProvider>
    );

    const toggle = screen.getByRole('button');

    // Check accessibility attributes
    expect(toggle).toHaveAttribute('aria-label');
    expect(toggle).toHaveAttribute('title');
    expect(toggle).toHaveAttribute('type', 'button');
  });
});