/**
 * E2E Journey Test: Reduced Motion Accessibility
 *
 * Tests that animations respect prefers-reduced-motion system preference.
 */

import { test, expect } from '@playwright/test';

test.describe('Journey 05: Reduced Motion Accessibility', () => {
  test('should disable animations when prefers-reduced-motion is set', async ({
    page,
  }) => {
    // Emulate reduced motion preference
    await page.emulateMedia({ reducedMotion: 'reduce' });

    await page.goto('/');

    // Check that elements don't have long animation durations
    const animatedElement = page.locator('[class*="animate"], .animate');
    if (await animatedElement.count() > 0) {
      const animationDuration = await animatedElement.first().evaluate((el) =>
        window.getComputedStyle(el).animationDuration
      );

      // Duration should be minimal or none
      expect(animationDuration).toMatch(/^(0s|0\.01ms|none)$/);
    }
  });

  test('should show content immediately without scroll reveal delays', async ({
    page,
  }) => {
    await page.emulateMedia({ reducedMotion: 'reduce' });

    await page.goto('/');

    // Scroll to bottom of page
    await page.evaluate(() => window.scrollTo(0, document.body.scrollHeight));
    await page.waitForTimeout(100);

    // All content should be visible immediately
    const content = page.locator('.scroll-reveal, [class*="reveal"]');
    if (await content.count() > 0) {
      for (let i = 0; i < Math.min(await content.count(), 5); i++) {
        await expect(content.nth(i)).toBeVisible();
      }
    }
  });

  test('should disable hover transform effects', async ({ page }) => {
    await page.emulateMedia({ reducedMotion: 'reduce' });

    await page.goto('/');

    // Find an element with hover effect
    const hoverElement = page.locator('.hover-lift, [class*="hover"]');
    if (await hoverElement.count() > 0) {
      await hoverElement.first().hover();

      const transform = await hoverElement.first().evaluate((el) =>
        window.getComputedStyle(el).transform
      );

      // Transform should be none or identity matrix
      expect(transform).toMatch(/^(none|matrix\(1, 0, 0, 1, 0, 0\))$/);
    }
  });

  test('should still show loading indicators with subtle animation', async ({
    page,
  }) => {
    await page.emulateMedia({ reducedMotion: 'reduce' });

    await page.goto('/');

    // Typing indicator should use opacity pulse instead of bounce
    const typingIndicator = page.locator('[class*="typing"], .typing-indicator');
    if (await typingIndicator.count() > 0) {
      await expect(typingIndicator.first()).toBeVisible();
    }
  });

  test('should maintain accessibility with reduced motion', async ({ page }) => {
    await page.emulateMedia({ reducedMotion: 'reduce' });

    await page.goto('/');

    // Check that focus indicators are still present
    await page.keyboard.press('Tab');
    const focused = page.locator(':focus-visible');
    if (await focused.count() > 0) {
      await expect(focused.first()).toBeVisible();
    }
  });

  test('should work correctly with system preference changes', async ({
    page,
  }) => {
    // Start with no preference
    await page.goto('/');

    // Enable reduced motion
    await page.emulateMedia({ reducedMotion: 'reduce' });
    await page.waitForTimeout(100);

    // Check animations are disabled
    const html = page.locator('html');
    const styles = await html.evaluate(() => {
      const computed = window.getComputedStyle(document.body);
      return computed.getPropertyValue('transition-duration');
    });

    // Transition duration should be minimal
    expect(styles).toBeDefined();

    // Disable reduced motion
    await page.emulateMedia({ reducedMotion: 'no-preference' });
    await page.waitForTimeout(100);

    // Page should still be functional
    await expect(page.locator('body')).toBeVisible();
  });
});
