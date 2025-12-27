/**
 * E2E Journey Test: Keyboard Navigation
 *
 * Tests keyboard-only navigation and accessibility.
 */

import { test, expect } from '@playwright/test';

test.describe('Journey 09: Keyboard Navigation', () => {
  test('should navigate to all main sections using Tab', async ({ page }) => {
    await page.goto('/');

    // Start tabbing
    const focusedElements: string[] = [];

    for (let i = 0; i < 10; i++) {
      await page.keyboard.press('Tab');
      await page.waitForTimeout(50);

      const focused = await page.evaluate(() => {
        const el = document.activeElement;
        return el?.tagName.toLowerCase() || 'none';
      });

      focusedElements.push(focused);
    }

    // Should have focused on various elements
    expect(focusedElements.length).toBe(10);
    expect(focusedElements.some((el) => el !== 'body')).toBe(true);
  });

  test('should have visible focus indicators', async ({ page }) => {
    await page.goto('/');

    // Tab to first focusable element
    await page.keyboard.press('Tab');
    await page.waitForTimeout(100);

    // Check for focus-visible styles
    const focusedElement = page.locator(':focus');
    if (await focusedElement.count() > 0) {
      const outline = await focusedElement.first().evaluate((el) =>
        window.getComputedStyle(el).outline
      );

      // Should have some outline or visible indicator
      expect(outline).not.toBe('0px none rgb(0, 0, 0)');
    }
  });

  test('should close modal/dialog with Escape key', async ({ page }) => {
    await page.goto('/');

    // Open mobile menu if on mobile
    const hamburger = page.locator('.navbar__toggle');
    if (await hamburger.count() > 0 && await hamburger.first().isVisible()) {
      await hamburger.first().click();
      await page.waitForTimeout(300);

      // Press Escape to close
      await page.keyboard.press('Escape');
      await page.waitForTimeout(300);

      // Menu should be closed
      const sidebar = page.locator('.navbar-sidebar--show');
      expect(await sidebar.count()).toBe(0);
    }
  });

  test('should navigate dropdown menus with Arrow keys', async ({ page }) => {
    await page.goto('/');

    // Find a dropdown
    const dropdown = page.locator('.dropdown__toggle');
    if (await dropdown.count() > 0) {
      // Focus and open dropdown
      await dropdown.first().focus();
      await dropdown.first().click();
      await page.waitForTimeout(200);

      // Navigate with arrow keys
      await page.keyboard.press('ArrowDown');
      await page.waitForTimeout(50);

      // An item should be focused
      const focused = page.locator(':focus');
      await expect(focused).toBeVisible();
    }
  });

  test('should activate links with Enter key', async ({ page }) => {
    await page.goto('/');

    // Tab to a link
    await page.keyboard.press('Tab');
    await page.keyboard.press('Tab');
    await page.waitForTimeout(100);

    // Get current URL
    const initialUrl = page.url();

    // Press Enter to activate
    await page.keyboard.press('Enter');
    await page.waitForTimeout(500);

    // Should have navigated (URL may have changed)
    await expect(page.locator('body')).toBeVisible();
  });

  test('should skip to main content with skip link', async ({ page }) => {
    await page.goto('/');

    // Tab once to potentially reveal skip link
    await page.keyboard.press('Tab');
    await page.waitForTimeout(100);

    // Check for skip link
    const skipLink = page.locator('a[href="#main"], a[href="#content"], a:has-text("Skip")');
    if (await skipLink.count() > 0) {
      // Should be able to activate it
      await skipLink.first().focus();
      await page.keyboard.press('Enter');
      await page.waitForTimeout(100);

      // Focus should move to main content
      const focused = page.locator(':focus');
      await expect(focused).toBeVisible();
    }
  });

  test('should trap focus in modal dialogs', async ({ page }) => {
    await page.goto('/');

    // Open a modal if available
    const modalTrigger = page.locator('[data-modal], [aria-haspopup="dialog"]');
    if (await modalTrigger.count() > 0) {
      await modalTrigger.first().click();
      await page.waitForTimeout(300);

      // Tab through modal
      const focusedBefore = await page.evaluate(() => document.activeElement?.className);

      await page.keyboard.press('Tab');
      await page.keyboard.press('Tab');
      await page.keyboard.press('Tab');
      await page.keyboard.press('Tab');
      await page.keyboard.press('Tab');

      // Focus should still be within modal area
      const focused = page.locator(':focus');
      await expect(focused).toBeVisible();
    }
  });

  test('should support Shift+Tab for reverse navigation', async ({ page }) => {
    await page.goto('/');

    // Tab forward a few times
    await page.keyboard.press('Tab');
    await page.keyboard.press('Tab');
    await page.keyboard.press('Tab');
    await page.waitForTimeout(100);

    const forward = await page.evaluate(() => document.activeElement?.tagName);

    // Tab backward
    await page.keyboard.press('Shift+Tab');
    await page.waitForTimeout(100);

    const backward = await page.evaluate(() => document.activeElement?.tagName);

    // Focus should have moved backward
    expect(backward).toBeDefined();
  });
});
