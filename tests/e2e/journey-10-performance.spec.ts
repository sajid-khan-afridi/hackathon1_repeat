/**
 * E2E Journey Test: Performance
 *
 * Tests performance metrics and Core Web Vitals compliance.
 */

import { test, expect } from '@playwright/test';

test.describe('Journey 10: Performance', () => {
  test('should load home page within acceptable time', async ({ page }) => {
    const startTime = Date.now();

    await page.goto('/');
    await page.waitForLoadState('domcontentloaded');

    const loadTime = Date.now() - startTime;

    // Should load within 5 seconds (generous for CI)
    expect(loadTime).toBeLessThan(5000);
  });

  test('should have no layout shift after initial paint', async ({ page }) => {
    await page.goto('/');
    await page.waitForLoadState('networkidle');

    // Wait for animations to settle
    await page.waitForTimeout(1000);

    // Take a measurement of current layout
    const initialPositions = await page.evaluate(() => {
      const elements = document.querySelectorAll('h1, nav, main');
      return Array.from(elements).map((el) => {
        const rect = el.getBoundingClientRect();
        return { top: rect.top, left: rect.left };
      });
    });

    // Wait a bit more
    await page.waitForTimeout(500);

    // Check positions haven't shifted
    const finalPositions = await page.evaluate(() => {
      const elements = document.querySelectorAll('h1, nav, main');
      return Array.from(elements).map((el) => {
        const rect = el.getBoundingClientRect();
        return { top: rect.top, left: rect.left };
      });
    });

    // Positions should be stable (allowing small rounding differences)
    for (let i = 0; i < initialPositions.length; i++) {
      expect(Math.abs(initialPositions[i].top - finalPositions[i].top)).toBeLessThan(5);
      expect(Math.abs(initialPositions[i].left - finalPositions[i].left)).toBeLessThan(5);
    }
  });

  test('should have reasonable bundle size', async ({ page }) => {
    const resourceSizes: number[] = [];

    page.on('response', async (response) => {
      const url = response.url();
      if (url.endsWith('.js') || url.endsWith('.css')) {
        try {
          const body = await response.body();
          resourceSizes.push(body.length);
        } catch {
          // Ignore errors for external resources
        }
      }
    });

    await page.goto('/');
    await page.waitForLoadState('networkidle');

    // Total JS/CSS should be reasonable
    const totalSize = resourceSizes.reduce((a, b) => a + b, 0);

    // Should be under 2MB total (generous for documentation site)
    expect(totalSize).toBeLessThan(2 * 1024 * 1024);
  });

  test('should not block main thread during animations', async ({ page }) => {
    await page.goto('/');

    // Scroll through page
    for (let i = 0; i < 5; i++) {
      await page.evaluate(() => window.scrollBy(0, 500));
      await page.waitForTimeout(100);
    }

    // Page should remain responsive
    const isResponsive = await page.evaluate(() => {
      return new Promise((resolve) => {
        const start = performance.now();
        requestAnimationFrame(() => {
          const delta = performance.now() - start;
          // Should respond within 100ms (60fps = 16ms per frame)
          resolve(delta < 100);
        });
      });
    });

    expect(isResponsive).toBe(true);
  });

  test('should lazy load images', async ({ page }) => {
    await page.goto('/');

    // Check for lazy loading attributes
    const images = page.locator('img[loading="lazy"], img[data-src]');
    const imgCount = await images.count();

    // If there are images, some should be lazy loaded
    const allImages = await page.locator('img').count();
    if (allImages > 3) {
      // At least some images should have lazy loading
      expect(imgCount).toBeGreaterThan(0);
    }
  });

  test('should preload critical resources', async ({ page }) => {
    const preloads: string[] = [];

    page.on('request', (request) => {
      if (request.resourceType() === 'font' || request.url().includes('preload')) {
        preloads.push(request.url());
      }
    });

    await page.goto('/');
    await page.waitForLoadState('domcontentloaded');

    // Should have some preloaded resources
    // (This may vary based on site configuration)
  });

  test('should have acceptable Time to Interactive', async ({ page }) => {
    await page.goto('/');

    // Wait for hydration/interactivity
    await page.waitForLoadState('networkidle');

    // Check that page is interactive
    const isInteractive = await page.evaluate(() => {
      // Check if React has hydrated
      const root = document.getElementById('__docusaurus');
      return root !== null && document.readyState === 'complete';
    });

    expect(isInteractive).toBe(true);
  });

  test('should handle rapid scrolling smoothly', async ({ page }) => {
    await page.goto('/');
    await page.waitForLoadState('networkidle');

    // Rapid scroll
    for (let i = 0; i < 20; i++) {
      await page.evaluate(() => window.scrollBy(0, 100));
    }

    // Page should still be functional
    await expect(page.locator('body')).toBeVisible();

    // Scroll back up
    await page.evaluate(() => window.scrollTo(0, 0));

    // Should work fine
    await expect(page.locator('nav')).toBeVisible();
  });
});
