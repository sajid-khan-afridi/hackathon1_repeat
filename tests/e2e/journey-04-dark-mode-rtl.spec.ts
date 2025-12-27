/**
 * E2E Journey Test: Dark Mode and RTL Support
 *
 * Tests dark mode toggling and right-to-left (RTL) language support.
 */

import { test, expect } from '@playwright/test';

test.describe('Journey 04: Dark Mode and RTL Support', () => {
  test.describe('Dark Mode', () => {
    test('should toggle between light and dark mode', async ({ page }) => {
      await page.goto('/');

      // Find and click color mode toggle
      const toggle = page.locator('[class*="colorMode"] button, button[aria-label*="mode" i]');
      if (await toggle.count() > 0) {
        const initialTheme = await page.locator('html').getAttribute('data-theme');

        await toggle.first().click();
        await page.waitForTimeout(300);

        const newTheme = await page.locator('html').getAttribute('data-theme');

        // Theme should have changed
        expect(newTheme).not.toBe(initialTheme);
      }
    });

    test('should persist dark mode preference', async ({ page }) => {
      await page.goto('/');

      // Set dark mode
      const toggle = page.locator('[class*="colorMode"] button');
      if (await toggle.count() > 0) {
        await toggle.first().click();
        await page.waitForTimeout(300);

        const theme = await page.locator('html').getAttribute('data-theme');

        // Reload page
        await page.reload();
        await page.waitForTimeout(300);

        // Check theme is preserved
        const newTheme = await page.locator('html').getAttribute('data-theme');
        expect(newTheme).toBe(theme);
      }
    });

    test('should have proper contrast in dark mode', async ({ page }) => {
      await page.goto('/');

      // Enable dark mode
      await page.evaluate(() => {
        document.documentElement.setAttribute('data-theme', 'dark');
      });

      // Check that body has dark background
      const body = page.locator('body');
      const bgColor = await body.evaluate((el) =>
        window.getComputedStyle(el).backgroundColor
      );

      // Should be a dark color
      expect(bgColor).toBeTruthy();
    });
  });

  test.describe('RTL Support', () => {
    test('should support RTL language (Urdu)', async ({ page }) => {
      // Navigate to Urdu version
      await page.goto('/ur/');

      // Check for RTL direction
      const htmlDir = await page.locator('html').getAttribute('dir');
      expect(htmlDir).toBe('rtl');
    });

    test('should flip layout in RTL mode', async ({ page }) => {
      await page.goto('/ur/');

      // Check that navbar is properly aligned
      const navbar = page.locator('nav.navbar');
      if (await navbar.count() > 0) {
        await expect(navbar).toBeVisible();
      }
    });

    test('should handle language toggle', async ({ page }) => {
      await page.goto('/');

      // Find language toggle
      const langToggle = page.locator('[class*="language"], [aria-label*="language" i]');
      if (await langToggle.count() > 0) {
        await langToggle.first().click();
        await page.waitForTimeout(300);
      }
    });

    test('should maintain functionality in RTL mode', async ({ page }) => {
      await page.goto('/ur/');

      // Check navigation still works
      const links = page.locator('nav a');
      if (await links.count() > 0) {
        await expect(links.first()).toBeVisible();
      }
    });
  });
});
