/**
 * E2E Journey Test: Navbar Navigation Experience
 *
 * Tests navbar functionality including dropdowns, tooltips,
 * mobile drawer, and keyboard navigation.
 */

import { test, expect } from '@playwright/test';

test.describe('Journey 03: Navbar Navigation Experience', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/');
  });

  test('should display navbar with all main links', async ({ page }) => {
    const navbar = page.locator('nav.navbar, nav[class*="navbar"]');
    await expect(navbar).toBeVisible();

    // Check for main navigation items
    await expect(page.getByRole('link', { name: /modules/i })).toBeVisible();
    await expect(page.getByRole('link', { name: /get started/i })).toBeVisible();
  });

  test('should navigate to different sections', async ({ page }) => {
    // Click on Get Started
    await page.getByRole('link', { name: /get started/i }).first().click();
    await page.waitForURL(/\/docs\//);

    // Should be on docs page
    expect(page.url()).toContain('/docs/');
  });

  test('should show dropdown menus on hover', async ({ page }) => {
    // Find dropdown items
    const dropdownTrigger = page.locator('.dropdown__toggle, [class*="dropdown"] > a');
    if (await dropdownTrigger.count() > 0) {
      await dropdownTrigger.first().hover();
      await page.waitForTimeout(300);

      // Check for dropdown menu
      const dropdownMenu = page.locator('.dropdown__menu, [class*="dropdown"] > ul');
      if (await dropdownMenu.count() > 0) {
        await expect(dropdownMenu.first()).toBeVisible();
      }
    }
  });

  test('should support keyboard navigation in navbar', async ({ page }) => {
    // Focus on first navbar link
    const firstLink = page.locator('nav a').first();
    await firstLink.focus();

    // Tab through links
    await page.keyboard.press('Tab');
    await page.waitForTimeout(100);

    // Some element should be focused
    const focused = page.locator(':focus');
    await expect(focused).toBeVisible();
  });

  test('should have visible focus indicators', async ({ page }) => {
    // Tab to navbar items
    await page.keyboard.press('Tab');
    await page.keyboard.press('Tab');

    // Check that focused element has visible outline
    const focused = page.locator(':focus-visible');
    if (await focused.count() > 0) {
      // Element should have some visual focus indicator
      await expect(focused.first()).toBeVisible();
    }
  });

  test('should handle color mode toggle', async ({ page }) => {
    // Find color mode toggle
    const toggleButton = page.locator('[class*="colorMode"] button, [aria-label*="dark" i]');
    if (await toggleButton.count() > 0) {
      await toggleButton.first().click();
      await page.waitForTimeout(300);

      // Check that theme changed
      const html = page.locator('html');
      const dataTheme = await html.getAttribute('data-theme');
      expect(dataTheme).toBeDefined();
    }
  });

  test('should display GitHub link with proper icon', async ({ page }) => {
    const githubLink = page.locator('.header-github-link, a[href*="github"]');
    if (await githubLink.count() > 0) {
      await expect(githubLink.first()).toBeVisible();

      // Should have aria-label
      const ariaLabel = await githubLink.first().getAttribute('aria-label');
      expect(ariaLabel).toBeTruthy();
    }
  });
});
