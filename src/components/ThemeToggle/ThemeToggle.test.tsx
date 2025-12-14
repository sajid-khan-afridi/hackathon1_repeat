import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import ThemeToggle from './index';

// Mock the hooks
jest.mock('@docusaurus/useIsBrowser');
jest.mock('@docusaurus/theme-common', () => ({
  useColorMode: jest.fn(),
}));

// Import the mocked hooks
import useIsBrowser from '@docusaurus/useIsBrowser';
import { useColorMode } from '@docusaurus/theme-common';

describe('ThemeToggle', () => {
  const mockSetColorMode = jest.fn();

  beforeEach(() => {
    jest.clearAllMocks();
    (useIsBrowser as jest.Mock).mockReturnValue(true);
    (useColorMode as jest.Mock).mockReturnValue({
      colorMode: 'light',
      setColorMode: mockSetColorMode,
    });
  });

  it('renders null on server side', () => {
    (useIsBrowser as jest.Mock).mockReturnValue(false);

    const { container } = render(<ThemeToggle />);
    expect(container.firstChild).toBeNull();
  });

  it('shows sun icon in dark mode', () => {
    (useColorMode as jest.Mock).mockReturnValue({
      colorMode: 'dark',
      setColorMode: mockSetColorMode,
    });

    render(<ThemeToggle />);

    const toggle = screen.getByRole('button', { name: /Switch to light mode/i });
    expect(toggle).toBeInTheDocument();

    // Check for sun icon (should be present in dark mode)
    const icon = toggle.querySelector('svg');
    expect(icon).toBeInTheDocument();
  });

  it('shows moon icon in light mode', () => {
    (useColorMode as jest.Mock).mockReturnValue({
      colorMode: 'light',
      setColorMode: mockSetColorMode,
    });

    render(<ThemeToggle />);

    const toggle = screen.getByRole('button', { name: /Switch to dark mode/i });
    expect(toggle).toBeInTheDocument();

    // Check for moon icon (should be present in light mode)
    const icon = toggle.querySelector('svg');
    expect(icon).toBeInTheDocument();
  });

  it('calls setColorMode when clicked in light mode', () => {
    (useColorMode as jest.Mock).mockReturnValue({
      colorMode: 'light',
      setColorMode: mockSetColorMode,
    });

    render(<ThemeToggle />);

    const toggle = screen.getByRole('button');
    fireEvent.click(toggle);

    expect(mockSetColorMode).toHaveBeenCalledWith('dark');
  });

  it('calls setColorMode when clicked in dark mode', () => {
    (useColorMode as jest.Mock).mockReturnValue({
      colorMode: 'dark',
      setColorMode: mockSetColorMode,
    });

    render(<ThemeToggle />);

    const toggle = screen.getByRole('button');
    fireEvent.click(toggle);

    expect(mockSetColorMode).toHaveBeenCalledWith('light');
  });

  it('has proper accessibility attributes', () => {
    render(<ThemeToggle />);

    const toggle = screen.getByRole('button');

    // Check accessibility attributes
    expect(toggle).toHaveAttribute('aria-label');
    expect(toggle).toHaveAttribute('title');
    expect(toggle).toHaveAttribute('type', 'button');
  });
});
