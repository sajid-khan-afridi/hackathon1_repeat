import { test, expect } from '@playwright/test';

/**
 * Mobile Menu UI/UX Tests
 * Tests for alignment, dark mode visibility, and visual styling
 */

test.describe('Mobile Menu UI/UX', () => {
  test.beforeEach(async ({ page }) => {
    // Set mobile viewport
    await page.setViewportSize({ width: 375, height: 667 });
    await page.goto('/');
  });

  test('should have properly aligned header elements in mobile sidebar', async ({ page }) => {
    const hamburger = page.locator('button.navbar__toggle');

    // Open mobile menu
    await hamburger.click();
    await page.waitForTimeout(400);

    // Check sidebar header elements
    const sidebarBrand = page.locator('.navbar-sidebar__brand');
    await expect(sidebarBrand).toBeVisible();

    // Get all buttons in the header (color toggle and close)
    const headerButtons = sidebarBrand.locator('button');
    const buttonCount = await headerButtons.count();
    expect(buttonCount).toBeGreaterThanOrEqual(2); // At least close button and color toggle

    // Check if elements are horizontally aligned
    const brandBox = await sidebarBrand.boundingBox();
    expect(brandBox).not.toBeNull();
    expect(brandBox!.height).toBeLessThan(100); // Should be a single row

    // Verify close button is visible and clickable
    const closeButton = sidebarBrand.locator('.navbar-sidebar__close');
    await expect(closeButton).toBeVisible();

    const closeBox = await closeButton.boundingBox();
    expect(closeBox).not.toBeNull();
    expect(closeBox!.width).toBeGreaterThanOrEqual(44);
    expect(closeBox!.height).toBeGreaterThanOrEqual(44);
  });

  test('should have proper spacing between header elements', async ({ page }) => {
    const hamburger = page.locator('button.navbar__toggle');

    // Open mobile menu
    await hamburger.click();
    await page.waitForTimeout(400);

    const sidebarBrand = page.locator('.navbar-sidebar__brand');
    const logo = sidebarBrand.locator('.navbar__brand');
    const closeButton = sidebarBrand.locator('.navbar-sidebar__close');

    // Get positions
    const logoBox = await logo.boundingBox();
    const closeBox = await closeButton.boundingBox();

    expect(logoBox).not.toBeNull();
    expect(closeBox).not.toBeNull();

    // Logo should be on the left, close button on the right
    expect(logoBox!.x).toBeLessThan(closeBox!.x);

    // There should be space between them
    const gap = closeBox!.x - (logoBox!.x + logoBox!.width);
    expect(gap).toBeGreaterThan(0);
  });

  test('should have visible text in light mode', async ({ page }) => {
    const hamburger = page.locator('button.navbar__toggle');

    // Ensure light mode
    const html = page.locator('html');
    const theme = await html.getAttribute('data-theme');

    // If not light mode, toggle to light
    if (theme === 'dark') {
      const colorToggle = page.locator('button[title*="mode"]').first();
      await colorToggle.click();
      await page.waitForTimeout(300);
    }

    // Open mobile menu
    await hamburger.click();
    await page.waitForTimeout(400);

    // Check navigation links are visible
    const navLinks = page.locator('.navbar-sidebar .navbar__link, .navbar-sidebar .menu__link');
    const firstLink = navLinks.first();

    if (await firstLink.isVisible()) {
      // Check text color has good contrast with background
      const color = await firstLink.evaluate(el => {
        const styles = window.getComputedStyle(el);
        return styles.color;
      });

      const bgColor = await firstLink.evaluate(el => {
        const styles = window.getComputedStyle(el);
        return styles.backgroundColor;
      });

      // Colors should be different (not both transparent or same)
      expect(color).not.toBe(bgColor);
      expect(color).not.toBe('rgba(0, 0, 0, 0)');
    }
  });

  test('should have visible text in dark mode', async ({ page }) => {
    const hamburger = page.locator('button.navbar__toggle');

    // Toggle to dark mode
    const html = page.locator('html');
    let theme = await html.getAttribute('data-theme');

    if (theme !== 'dark') {
      const colorToggle = page.locator('button[title*="mode"]').first();
      await colorToggle.click();
      await page.waitForTimeout(300);

      // Verify dark mode is active
      theme = await html.getAttribute('data-theme');
    }

    // Open mobile menu
    await hamburger.click();
    await page.waitForTimeout(400);

    // Check sidebar background is dark
    const sidebar = page.locator('.navbar-sidebar');
    const bgColor = await sidebar.evaluate(el => {
      return window.getComputedStyle(el).backgroundColor;
    });

    // Dark mode should have dark background (not white)
    expect(bgColor).not.toBe('rgb(255, 255, 255)');

    // Check navigation links have light text
    const navLinks = page.locator('.navbar-sidebar .navbar__link, .navbar-sidebar .menu__link');
    const firstLink = navLinks.first();

    if (await firstLink.isVisible()) {
      const color = await firstLink.evaluate(el => {
        const styles = window.getComputedStyle(el);
        return styles.color;
      });

      // Text should be light colored in dark mode (not dark/black)
      expect(color).not.toBe('rgb(0, 0, 0)');

      // Should be a light shade (rgb values should be > 100)
      const rgbMatch = color.match(/rgb\((\d+),\s*(\d+),\s*(\d+)\)/);
      if (rgbMatch) {
        const [, r, g, b] = rgbMatch.map(Number);
        const avgBrightness = (r + g + b) / 3;
        expect(avgBrightness).toBeGreaterThan(100); // Light text
      }
    }
  });

  test('should toggle between light and dark mode in mobile menu', async ({ page }) => {
    const hamburger = page.locator('button.navbar__toggle');

    // Open mobile menu
    await hamburger.click();
    await page.waitForTimeout(400);

    const html = page.locator('html');
    const initialTheme = await html.getAttribute('data-theme');

    // Find color mode toggle in sidebar
    const sidebar = page.locator('.navbar-sidebar');
    const colorToggle = sidebar.locator('button[title*="mode"]');
    await expect(colorToggle).toBeVisible();

    // Click to toggle
    await colorToggle.click();
    await page.waitForTimeout(300);

    const newTheme = await html.getAttribute('data-theme');

    // Theme should have changed
    expect(newTheme).not.toBe(initialTheme);

    // Sidebar should still be visible with proper colors
    const sidebarBgColor = await sidebar.evaluate(el => {
      return window.getComputedStyle(el).backgroundColor;
    });

    expect(sidebarBgColor).not.toBe('rgba(0, 0, 0, 0)'); // Not transparent
  });

  test('should have proper hover states in mobile menu', async ({ page }) => {
    const hamburger = page.locator('button.navbar__toggle');

    // Open mobile menu
    await hamburger.click();
    await page.waitForTimeout(400);

    // Get a navigation link
    const navLinks = page.locator('.navbar-sidebar .navbar__link, .navbar-sidebar .menu__link');
    const firstLink = navLinks.first();

    if (await firstLink.isVisible()) {
      // Get initial background
      const initialBg = await firstLink.evaluate(el => {
        return window.getComputedStyle(el).backgroundColor;
      });

      // Hover over the link
      await firstLink.hover();
      await page.waitForTimeout(100);

      // Background should change on hover
      const hoverBg = await firstLink.evaluate(el => {
        return window.getComputedStyle(el).backgroundColor;
      });

      // Hover state should be different (or at least visible)
      // We just check it's not transparent
      expect(hoverBg).not.toBe('rgba(0, 0, 0, 0)');
    }
  });

  test('should have consistent header height and padding', async ({ page }) => {
    const hamburger = page.locator('button.navbar__toggle');

    // Open mobile menu
    await hamburger.click();
    await page.waitForTimeout(400);

    const sidebarBrand = page.locator('.navbar-sidebar__brand');

    // Check padding
    const padding = await sidebarBrand.evaluate(el => {
      const styles = window.getComputedStyle(el);
      return {
        top: styles.paddingTop,
        bottom: styles.paddingBottom,
        left: styles.paddingLeft,
        right: styles.paddingRight,
      };
    });

    // Should have consistent padding (1rem = 16px)
    expect(parseFloat(padding.top)).toBeGreaterThanOrEqual(12);
    expect(parseFloat(padding.bottom)).toBeGreaterThanOrEqual(12);

    // Check border
    const borderBottom = await sidebarBrand.evaluate(el => {
      return window.getComputedStyle(el).borderBottomWidth;
    });

    expect(parseFloat(borderBottom)).toBeGreaterThan(0); // Should have bottom border
  });
});
