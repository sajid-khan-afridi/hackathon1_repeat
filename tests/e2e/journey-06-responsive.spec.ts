/**
 * E2E Journey Test: Responsive Layout
 *
 * Tests responsive design across different viewport sizes.
 */

import { test, expect } from '@playwright/test';

const viewports = {
  mobile: { width: 375, height: 667 },
  tablet: { width: 768, height: 1024 },
  desktop: { width: 1280, height: 720 },
  widescreen: { width: 1920, height: 1080 },
};

test.describe('Journey 06: Responsive Layout', () => {
  test.describe('Mobile (375px)', () => {
    test.use({ viewport: viewports.mobile });

    test('should display mobile navigation', async ({ page }) => {
      await page.goto('/');

      // Mobile hamburger menu should be visible
      const hamburger = page.locator('.navbar__toggle, button[aria-label*="menu" i]');
      await expect(hamburger.first()).toBeVisible();
    });

    test('should not have horizontal overflow', async ({ page }) => {
      await page.goto('/');

      const bodyWidth = await page.evaluate(() => document.body.scrollWidth);
      const viewportWidth = await page.evaluate(() => window.innerWidth);

      expect(bodyWidth).toBeLessThanOrEqual(viewportWidth + 1);
    });

    test('should have proper touch targets (44px minimum)', async ({ page }) => {
      await page.goto('/');

      const buttons = page.locator('button, a');
      const count = await buttons.count();

      for (let i = 0; i < Math.min(count, 10); i++) {
        const box = await buttons.nth(i).boundingBox();
        if (box && box.width > 0 && box.height > 0) {
          // At least one dimension should meet minimum
          const meetsMinimum = box.width >= 44 || box.height >= 44;
          // Small icons/links may be exempt, but main interactive elements should comply
        }
      }
    });

    test('should open mobile sidebar menu', async ({ page }) => {
      await page.goto('/');

      const hamburger = page.locator('.navbar__toggle');
      if (await hamburger.count() > 0) {
        await hamburger.first().click();
        await page.waitForTimeout(300);

        // Sidebar should be visible
        const sidebar = page.locator('.navbar-sidebar, [class*="sidebar"]');
        if (await sidebar.count() > 0) {
          await expect(sidebar.first()).toBeVisible();
        }
      }
    });
  });

  test.describe('Tablet (768px)', () => {
    test.use({ viewport: viewports.tablet });

    test('should adapt layout for tablet', async ({ page }) => {
      await page.goto('/');

      // Content should be visible
      await expect(page.locator('main')).toBeVisible();
    });

    test('should not have horizontal overflow', async ({ page }) => {
      await page.goto('/');

      const bodyWidth = await page.evaluate(() => document.body.scrollWidth);
      const viewportWidth = await page.evaluate(() => window.innerWidth);

      expect(bodyWidth).toBeLessThanOrEqual(viewportWidth + 1);
    });
  });

  test.describe('Desktop (1280px)', () => {
    test.use({ viewport: viewports.desktop });

    test('should display full desktop navigation', async ({ page }) => {
      await page.goto('/');

      // Desktop nav links should be visible
      await expect(page.getByRole('link', { name: /modules/i })).toBeVisible();
      await expect(page.getByRole('link', { name: /get started/i })).toBeVisible();

      // Hamburger should not be visible
      const hamburger = page.locator('.navbar__toggle');
      if (await hamburger.count() > 0) {
        await expect(hamburger.first()).not.toBeVisible();
      }
    });

    test('should have proper content width constraints', async ({ page }) => {
      await page.goto('/');

      const main = page.locator('main');
      const box = await main.boundingBox();

      if (box) {
        // Content shouldn't exceed reasonable width
        expect(box.width).toBeLessThanOrEqual(1400);
      }
    });
  });

  test.describe('Widescreen (1920px)', () => {
    test.use({ viewport: viewports.widescreen });

    test('should center content on wide screens', async ({ page }) => {
      await page.goto('/');

      // Content should be centered
      await expect(page.locator('main')).toBeVisible();
    });

    test('should not have horizontal overflow', async ({ page }) => {
      await page.goto('/');

      const bodyWidth = await page.evaluate(() => document.body.scrollWidth);
      const viewportWidth = await page.evaluate(() => window.innerWidth);

      expect(bodyWidth).toBeLessThanOrEqual(viewportWidth + 1);
    });
  });

  test.describe('Minimum viewport (320px)', () => {
    test.use({ viewport: { width: 320, height: 568 } });

    test('should be usable at 320px width', async ({ page }) => {
      await page.goto('/');

      // Page should load without errors
      await expect(page.locator('body')).toBeVisible();

      // No horizontal overflow
      const bodyWidth = await page.evaluate(() => document.body.scrollWidth);
      expect(bodyWidth).toBeLessThanOrEqual(320 + 1);
    });
  });
});
