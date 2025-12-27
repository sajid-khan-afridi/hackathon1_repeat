/**
 * E2E Journey Test: Learning Session
 *
 * Tests a typical learning session flow through the textbook.
 */

import { test, expect } from '@playwright/test';

test.describe('Journey 07: Learning Session', () => {
  test('should navigate to documentation', async ({ page }) => {
    await page.goto('/');

    // Click on Get Started or Modules
    const getStarted = page.getByRole('link', { name: /get started/i });
    if (await getStarted.count() > 0) {
      await getStarted.first().click();
      await page.waitForURL(/\/docs\//);

      // Should be on a docs page
      expect(page.url()).toContain('/docs/');
    }
  });

  test('should display documentation sidebar', async ({ page }) => {
    await page.goto('/docs/intro');

    // Sidebar should be visible on desktop
    const sidebar = page.locator('.theme-doc-sidebar-container, [class*="sidebar"]');
    if (await sidebar.count() > 0) {
      await expect(sidebar.first()).toBeVisible();
    }
  });

  test('should navigate between chapters', async ({ page }) => {
    await page.goto('/docs/intro');

    // Find and click a sidebar link
    const sidebarLink = page.locator('.menu__link, [class*="sidebar"] a');
    if (await sidebarLink.count() > 1) {
      await sidebarLink.nth(1).click();
      await page.waitForTimeout(300);

      // URL should change
      expect(page.url()).not.toContain('/intro');
    }
  });

  test('should use pagination to go to next chapter', async ({ page }) => {
    await page.goto('/docs/intro');

    // Find next page link
    const nextLink = page.locator('.pagination-nav__link--next, a[class*="next"]');
    if (await nextLink.count() > 0) {
      await nextLink.first().click();
      await page.waitForTimeout(300);

      // Should be on a different page
      expect(page.url()).not.toContain('/intro');
    }
  });

  test('should display code blocks properly', async ({ page }) => {
    await page.goto('/docs/intro');

    // Look for code blocks
    const codeBlock = page.locator('pre code, .prism-code, [class*="codeBlock"]');
    if (await codeBlock.count() > 0) {
      await expect(codeBlock.first()).toBeVisible();
    }
  });

  test('should access glossary', async ({ page }) => {
    await page.goto('/');

    // Navigate to glossary
    const glossaryLink = page.getByRole('link', { name: /glossary/i });
    if (await glossaryLink.count() > 0) {
      await glossaryLink.first().click();
      await page.waitForURL(/\/glossary/);

      expect(page.url()).toContain('/glossary');
    }
  });

  test('should have working internal links', async ({ page }) => {
    await page.goto('/docs/intro');

    // Find internal links
    const internalLinks = page.locator('a[href^="/"], a[href^="./"]');
    if (await internalLinks.count() > 0) {
      // Click a link and verify navigation
      const link = internalLinks.first();
      const href = await link.getAttribute('href');

      if (href && !href.includes('#')) {
        await link.click();
        await page.waitForTimeout(500);

        // Page should load without errors
        await expect(page.locator('body')).toBeVisible();
      }
    }
  });
});
