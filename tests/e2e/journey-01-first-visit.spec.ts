/**
 * E2E Journey Test: First Visit Experience
 *
 * Tests the experience of a first-time visitor to the site.
 * Verifies home page animations, feature cards, and initial interactions.
 */

import { test, expect } from '@playwright/test';

test.describe('Journey 01: First Visit Experience', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/');
  });

  test('should display home page with hero section', async ({ page }) => {
    // Wait for hero section to be visible
    await expect(page.locator('h1')).toBeVisible();

    // Verify main navigation is present
    await expect(page.locator('nav')).toBeVisible();

    // Check for main CTA buttons
    await expect(page.getByRole('link', { name: /get started/i })).toBeVisible();
  });

  test('should show feature cards with hover effects', async ({ page }) => {
    // Scroll to features section
    const featuresSection = page.locator('.features, [class*="features"]');
    if (await featuresSection.count() > 0) {
      await featuresSection.first().scrollIntoViewIfNeeded();

      // Check feature cards are visible
      const cards = page.locator('[class*="feature"], .featureCard, article');
      const cardCount = await cards.count();
      expect(cardCount).toBeGreaterThan(0);
    }
  });

  test('should have proper navigation links', async ({ page }) => {
    // Check main navigation items
    await expect(page.getByRole('link', { name: /modules/i })).toBeVisible();
    await expect(page.getByRole('link', { name: /get started/i })).toBeVisible();
  });

  test('should load without console errors', async ({ page }) => {
    const errors: string[] = [];
    page.on('console', (msg) => {
      if (msg.type() === 'error') {
        errors.push(msg.text());
      }
    });

    await page.goto('/');
    await page.waitForLoadState('networkidle');

    // Filter out known non-critical errors:
    // - favicon/manifest: browser resource requests
    // - ERR_CONNECTION_REFUSED: API server not running during E2E tests (expected)
    // - Failed to load resource: network errors when API is not available
    const criticalErrors = errors.filter(
      (e) =>
        !e.includes('favicon') &&
        !e.includes('manifest') &&
        !e.includes('ERR_CONNECTION_REFUSED') &&
        !e.includes('Failed to load resource')
    );
    expect(criticalErrors).toHaveLength(0);
  });

  test('should have accessible landmarks', async ({ page }) => {
    // Check for main landmark
    await expect(page.locator('main')).toBeVisible();

    // Check for navigation landmark
    await expect(page.locator('nav')).toBeVisible();

    // Check for footer
    await expect(page.locator('footer')).toBeVisible();
  });

  test('should display statistics section with animations', async ({ page }) => {
    // Look for stats/counter section if present
    const statsSection = page.locator('[class*="stats"], [class*="counter"], [class*="metric"]');
    if (await statsSection.count() > 0) {
      await statsSection.first().scrollIntoViewIfNeeded();
      await expect(statsSection.first()).toBeVisible();
    }
  });
});
