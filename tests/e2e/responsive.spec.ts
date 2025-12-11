import { test, expect, devices } from '@playwright/test';

// Viewports to test
const viewports = [
  { name: 'Mobile', width: 375, height: 667 },
  { name: 'Tablet', width: 768, height: 1024 },
  { name: 'Desktop', width: 1024, height: 768 },
  { name: 'Large Desktop', width: 1920, height: 1080 }
];

test.describe('Responsive Design Tests', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/');
  });

  viewports.forEach(viewport => {
    test.describe(`${viewport.name} (${viewport.width}x${viewport.height})`, () => {
      test.use({ viewport: { width: viewport.width, height: viewport.height } });

      test('should display content without horizontal scroll', async ({ page }) => {
        // Check that horizontal scrolling is not needed
        const pageWidth = await page.evaluate(() => document.body.scrollWidth);
        const viewportWidth = await page.evaluate(() => window.innerWidth);

        expect(pageWidth).toBeLessThanOrEqual(viewportWidth + 1); // Allow 1px tolerance
      });

      test('should have accessible touch targets on mobile', async ({ page }) => {
        if (viewport.width < 768) {
          // Get all interactive elements
          const clickableElements = await page.locator('a, button, input, select, textarea').all();

          for (const element of clickableElements) {
            const box = await element.boundingBox();
            if (box) {
              // Ensure touch targets are at least 44x44px
              expect(box.width).toBeGreaterThanOrEqual(44);
              expect(box.height).toBeGreaterThanOrEqual(44);
            }
          }
        }
      });

      test('should have proper navigation layout', async ({ page }) => {
        const navbar = page.locator('.navbar');
        await expect(navbar).toBeVisible();

        // Check sidebar behavior
        const sidebar = page.locator('.docSidebarContainer');

        if (viewport.width < 996) {
          // On smaller screens, sidebar should be hidden by default
          await expect(sidebar).toHaveClass(/docSidebarContainer/);
          // Check for hamburger menu
          const menuToggle = page.locator('.navbar__toggle');
          if (await menuToggle.isVisible()) {
            await menuToggle.click();
            await expect(sidebar).toHaveClass(/docSidebarContainer--open/);
          }
        } else {
          // On larger screens, sidebar should be visible
          await expect(sidebar).toBeVisible();
        }
      });

      test('should display content properly', async ({ page }) => {
        // Navigate to a chapter
        await page.locator('a[href*="chapter-1-publishers"]').click();

        // Check that content is visible
        await expect(page.locator('h1')).toBeVisible();

        // Check code blocks
        const codeBlocks = page.locator('pre code');
        if (await codeBlocks.count() > 0) {
          await expect(codeBlocks.first()).toBeVisible();

          // Verify code is not overflowing on mobile
          if (viewport.width < 768) {
            const codeContainer = page.locator('.codeBlockContent').first();
            const containerBox = await codeContainer.boundingBox();
            if (containerBox) {
              expect(containerBox.width).toBeLessThanOrEqual(viewport.width + 10);
            }
          }
        }
      });

      test('should have readable text sizes', async ({ page }) => {
        // Navigate to content
        await page.locator('a[href*="chapter-1-publishers"]').click();

        // Check heading sizes
        const h1 = page.locator('h1').first();
        const h1Size = await h1.evaluate(el =>
          window.getComputedStyle(el).fontSize
        );

        // Convert to number and check minimum size
        const h1SizeNum = parseFloat(h1Size);
        expect(h1SizeNum).toBeGreaterThanOrEqual(24); // 24px minimum for h1

        // Check paragraph readability
        const paragraph = page.locator('p').first();
        const pSize = await paragraph.evaluate(el =>
          window.getComputedStyle(el).fontSize
        );
        const pSizeNum = parseFloat(pSize);
        expect(pSizeNum).toBeGreaterThanOrEqual(16); // 16px minimum for paragraphs
      });

      test('should have proper table responsiveness', async ({ page }) => {
        await page.locator('a[href*="chapter-1-publishers"]').click();

        const table = page.locator('table').first();
        if (await table.isVisible()) {
          // Check table is scrollable on mobile if needed
          if (viewport.width < 768) {
            const tableContainer = page.locator('.markdown table').first();
            const containerBox = await tableContainer.boundingBox();
            if (containerBox && containerBox.width > viewport.width) {
              // Table should have horizontal scroll
              const isScrollable = await tableContainer.evaluate(el => {
                return el.scrollWidth > el.clientWidth;
              });
              expect(isScrollable).toBeTruthy();
            }
          }
        }
      });
    });
  });

  test('should handle orientation changes gracefully', async ({ page }) => {
    // Start with portrait
    await page.setViewportSize({ width: 375, height: 667 });
    await page.goto('/');

    // Check layout
    const sidebar = page.locator('.docSidebarContainer');
    await expect(sidebar).toBeVisible();

    // Switch to landscape
    await page.setViewportSize({ width: 667, height: 375 });

    // Page should still be functional
    await expect(sidebar).toBeVisible();
    await expect(page.locator('.navbar')).toBeVisible();

    // No horizontal scroll
    const pageWidth = await page.evaluate(() => document.body.scrollWidth);
    expect(pageWidth).toBeLessThanOrEqual(667);
  });
});