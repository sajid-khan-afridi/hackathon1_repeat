/**
 * E2E Journey Test: Error Recovery
 *
 * Tests error handling and recovery scenarios.
 */

import { test, expect } from '@playwright/test';

test.describe('Journey 08: Error Recovery', () => {
  test('should display 404 page for non-existent routes', async ({ page }) => {
    await page.goto('/this-page-does-not-exist-12345');

    // Should show 404 or redirect to home
    const body = page.locator('body');
    await expect(body).toBeVisible();

    // Either 404 content or home page
    const is404 = await page.locator('text=404, text=not found').count() > 0;
    const isHome = page.url().endsWith('/');

    expect(is404 || isHome).toBe(true);
  });

  test('should handle navigation to broken anchor links', async ({ page }) => {
    await page.goto('/docs/intro#non-existent-anchor');

    // Page should still load
    await expect(page.locator('body')).toBeVisible();

    // Should be on the docs page
    expect(page.url()).toContain('/docs/intro');
  });

  test('should recover from rapid navigation', async ({ page }) => {
    // Rapid navigation between pages
    await page.goto('/');
    await page.goto('/docs/intro');
    await page.goto('/glossary');
    await page.goto('/');

    // Should end up on home page without errors
    await expect(page.locator('body')).toBeVisible();
  });

  test('should handle back/forward navigation', async ({ page }) => {
    await page.goto('/');
    await page.goto('/docs/intro');

    // Go back
    await page.goBack();
    await page.waitForTimeout(300);

    // Should be on home page
    expect(page.url()).toMatch(/\/$/);

    // Go forward
    await page.goForward();
    await page.waitForTimeout(300);

    // Should be on docs page
    expect(page.url()).toContain('/docs/intro');
  });

  test('should handle network slowdown gracefully', async ({ page }) => {
    // Simulate slow network
    await page.route('**/*', async (route) => {
      await new Promise((resolve) => setTimeout(resolve, 100));
      await route.continue();
    });

    await page.goto('/');

    // Page should still load
    await expect(page.locator('body')).toBeVisible();
  });

  test('should handle refresh during navigation', async ({ page }) => {
    await page.goto('/docs/intro');

    // Reload page
    await page.reload();

    // Should reload successfully
    await expect(page.locator('body')).toBeVisible();
    expect(page.url()).toContain('/docs/intro');
  });

  test('should maintain state after page refresh', async ({ page }) => {
    await page.goto('/');

    // Toggle dark mode
    const toggle = page.locator('[class*="colorMode"] button');
    if (await toggle.count() > 0) {
      await toggle.first().click();
      await page.waitForTimeout(300);

      const theme = await page.locator('html').getAttribute('data-theme');

      // Refresh
      await page.reload();
      await page.waitForTimeout(300);

      // Theme should persist
      const newTheme = await page.locator('html').getAttribute('data-theme');
      expect(newTheme).toBe(theme);
    }
  });
});
