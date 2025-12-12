import { test, expect, devices } from '@playwright/test';

/**
 * Mobile Hamburger Menu E2E Tests
 *
 * Tests the mobile hamburger menu functionality across different viewports
 * to ensure proper visibility, interaction, and accessibility.
 */

// Mobile viewports to test
const mobileViewports = [
  { name: 'iPhone SE', width: 375, height: 667 },
  { name: 'iPhone 12', width: 390, height: 844 },
  { name: 'Samsung Galaxy S21', width: 360, height: 800 },
  { name: 'iPad Mini', width: 768, height: 1024 },
];

test.describe('Mobile Hamburger Menu', () => {

  mobileViewports.forEach(viewport => {
    test.describe(`${viewport.name} (${viewport.width}x${viewport.height})`, () => {
      test.use({ viewport: { width: viewport.width, height: viewport.height } });

      test.beforeEach(async ({ page }) => {
        await page.goto('/');
      });

      test('should display hamburger menu button on mobile', async ({ page }) => {
        // Check if viewport is mobile (≤996px)
        if (viewport.width <= 996) {
          const hamburger = page.locator('button.navbar__toggle');

          // Button should be visible
          await expect(hamburger).toBeVisible();

          // Button should have proper ARIA attributes
          await expect(hamburger).toHaveAttribute('aria-label', /toggle navigation/i);
          await expect(hamburger).toHaveAttribute('aria-expanded');
          await expect(hamburger).toHaveAttribute('type', 'button');

          // Check if icon is visible
          const icon = hamburger.locator('svg');
          await expect(icon).toBeVisible();

          // Verify button is clickable (has proper dimensions)
          const box = await hamburger.boundingBox();
          expect(box).not.toBeNull();
          expect(box!.width).toBeGreaterThanOrEqual(44); // Minimum touch target
          expect(box!.height).toBeGreaterThanOrEqual(44);
        }
      });

      test('should toggle mobile sidebar when hamburger is clicked', async ({ page }) => {
        if (viewport.width <= 996) {
          const hamburger = page.locator('button.navbar__toggle');
          const sidebar = page.locator('.navbar-sidebar');

          // Initial state - sidebar should not be visible or should be off-screen
          const initialAriaExpanded = await hamburger.getAttribute('aria-expanded');
          expect(initialAriaExpanded).toBe('false');

          // Click hamburger to open
          await hamburger.click();

          // Wait for animation
          await page.waitForTimeout(400);

          // Sidebar should be visible/open
          await expect(sidebar).toBeVisible();
          const ariaExpandedAfterOpen = await hamburger.getAttribute('aria-expanded');
          expect(ariaExpandedAfterOpen).toBe('true');

          // Check if navbar has the show class
          const navbar = page.locator('nav.navbar');
          await expect(navbar).toHaveClass(/navbar-sidebar--show/);

          // Backdrop should be visible
          const backdrop = page.locator('.navbar-sidebar__backdrop');
          await expect(backdrop).toBeVisible();
        }
      });

      test('should display navigation items in mobile sidebar', async ({ page }) => {
        if (viewport.width <= 996) {
          const hamburger = page.locator('button.navbar__toggle');

          // Open mobile menu
          await hamburger.click();
          await page.waitForTimeout(400);

          // Check for navigation items in sidebar
          const sidebar = page.locator('.navbar-sidebar');

          // Verify "Modules" link exists
          const modulesLink = sidebar.locator('a', { hasText: /modules/i });
          await expect(modulesLink.first()).toBeVisible();

          // Verify "Get Started" link exists
          const getStartedLink = sidebar.locator('a', { hasText: /get started/i });
          await expect(getStartedLink.first()).toBeVisible();

          // Verify all links are properly styled and accessible
          const allLinks = sidebar.locator('a.navbar__link, a.menu__link');
          const linkCount = await allLinks.count();
          expect(linkCount).toBeGreaterThan(0);

          // Check first link has proper touch target size
          const firstLinkBox = await allLinks.first().boundingBox();
          expect(firstLinkBox).not.toBeNull();
          expect(firstLinkBox!.height).toBeGreaterThanOrEqual(44);
        }
      });

      test('should close mobile sidebar when backdrop is clicked', async ({ page }) => {
        if (viewport.width <= 996) {
          const hamburger = page.locator('button.navbar__toggle');
          const backdrop = page.locator('.navbar-sidebar__backdrop');

          // Open menu
          await hamburger.click();
          await page.waitForTimeout(400);

          // Verify menu is open
          const ariaExpanded = await hamburger.getAttribute('aria-expanded');
          expect(ariaExpanded).toBe('true');

          // Click backdrop to close
          await backdrop.click();
          await page.waitForTimeout(400);

          // Menu should be closed
          const ariaExpandedAfter = await hamburger.getAttribute('aria-expanded');
          expect(ariaExpandedAfter).toBe('false');
        }
      });

      test('should close mobile sidebar when close button is clicked', async ({ page }) => {
        if (viewport.width <= 996) {
          const hamburger = page.locator('button.navbar__toggle');

          // Open menu
          await hamburger.click();
          await page.waitForTimeout(400);

          // Find and click close button in sidebar
          const sidebar = page.locator('.navbar-sidebar');
          const closeButton = sidebar.locator('button[aria-label*="Close"], button.close, .navbar-sidebar__close');

          if (await closeButton.count() > 0) {
            await closeButton.first().click();
            await page.waitForTimeout(400);

            // Menu should be closed
            const ariaExpanded = await hamburger.getAttribute('aria-expanded');
            expect(ariaExpanded).toBe('false');
          }
        }
      });

      test('should have proper CSS styling for hamburger button', async ({ page }) => {
        if (viewport.width <= 996) {
          const hamburger = page.locator('button.navbar__toggle');

          // Check computed styles
          const display = await hamburger.evaluate(el =>
            window.getComputedStyle(el).display
          );
          expect(display).not.toBe('none');

          const visibility = await hamburger.evaluate(el =>
            window.getComputedStyle(el).visibility
          );
          expect(visibility).toBe('visible');

          const opacity = await hamburger.evaluate(el =>
            window.getComputedStyle(el).opacity
          );
          expect(parseFloat(opacity)).toBeGreaterThan(0);

          // Check icon styling
          const icon = hamburger.locator('svg');
          const iconDisplay = await icon.evaluate(el =>
            window.getComputedStyle(el).display
          );
          expect(iconDisplay).not.toBe('none');
        }
      });

      test('should hide desktop navigation items on mobile', async ({ page }) => {
        if (viewport.width <= 996) {
          // Desktop nav items should be hidden on mobile
          const navbarItems = page.locator('.navbar__items--right > .navbar__item:not(.navbar__toggle)');
          const itemCount = await navbarItems.count();

          // Check if items are hidden
          for (let i = 0; i < itemCount; i++) {
            const item = navbarItems.nth(i);
            const display = await item.evaluate(el =>
              window.getComputedStyle(el).display
            );
            // Items should be hidden on mobile
            expect(display).toBe('none');
          }
        }
      });

      test('should maintain hamburger button visibility on scroll', async ({ page }) => {
        if (viewport.width <= 996) {
          const hamburger = page.locator('button.navbar__toggle');

          // Verify button is visible before scroll
          await expect(hamburger).toBeVisible();

          // Scroll down the page
          await page.evaluate(() => window.scrollBy(0, 500));
          await page.waitForTimeout(300);

          // Button should still be visible (navbar is fixed)
          await expect(hamburger).toBeVisible();

          // Scroll back up
          await page.evaluate(() => window.scrollTo(0, 0));
          await page.waitForTimeout(300);

          // Button should still be visible
          await expect(hamburger).toBeVisible();
        }
      });

      test('should have proper keyboard accessibility', async ({ page }) => {
        if (viewport.width <= 996) {
          const hamburger = page.locator('button.navbar__toggle');

          // Focus on hamburger button
          await hamburger.focus();

          // Check if button is focused
          const isFocused = await hamburger.evaluate(el =>
            document.activeElement === el
          );
          expect(isFocused).toBeTruthy();

          // Press Enter to open menu
          await page.keyboard.press('Enter');
          await page.waitForTimeout(400);

          // Menu should be open
          const ariaExpanded = await hamburger.getAttribute('aria-expanded');
          expect(ariaExpanded).toBe('true');

          // Press Escape to close menu
          await page.keyboard.press('Escape');
          await page.waitForTimeout(400);

          // Menu should be closed
          const ariaExpandedAfter = await hamburger.getAttribute('aria-expanded');
          expect(ariaExpandedAfter).toBe('false');
        }
      });

      test('should lock body scroll when mobile menu is open', async ({ page }) => {
        if (viewport.width <= 996) {
          const hamburger = page.locator('button.navbar__toggle');

          // Check body overflow before opening menu
          const overflowBefore = await page.evaluate(() =>
            window.getComputedStyle(document.body).overflow
          );

          // Open menu
          await hamburger.click();
          await page.waitForTimeout(400);

          // Body should have overflow hidden when menu is open
          const overflowAfter = await page.evaluate(() =>
            window.getComputedStyle(document.body).overflow
          );

          // Docusaurus uses useLockBodyScroll hook
          // Body scroll should be locked when sidebar is open
          expect(overflowAfter).toBe('hidden');

          // Close menu
          const backdrop = page.locator('.navbar-sidebar__backdrop');
          await backdrop.click();
          await page.waitForTimeout(400);

          // Body overflow should be restored
          const overflowRestored = await page.evaluate(() =>
            window.getComputedStyle(document.body).overflow
          );
          expect(overflowRestored).not.toBe('hidden');
        }
      });
    });
  });

  test('should NOT display hamburger on desktop', async ({ page }) => {
    // Set desktop viewport
    await page.setViewportSize({ width: 1920, height: 1080 });
    await page.goto('/');

    // Hamburger should not be visible on desktop
    const hamburger = page.locator('button.navbar__toggle');

    // Check if button exists but is hidden
    const display = await hamburger.evaluate(el =>
      window.getComputedStyle(el).display
    ).catch(() => 'none');

    // On desktop, hamburger should be hidden
    expect(display).toBe('none');
  });

  test('should handle rapid open/close clicks gracefully', async ({ page }) => {
    await page.setViewportSize({ width: 375, height: 667 });
    await page.goto('/');

    const hamburger = page.locator('button.navbar__toggle');

    // Rapid clicks
    await hamburger.click();
    await page.waitForTimeout(100);
    await hamburger.click();
    await page.waitForTimeout(100);
    await hamburger.click();
    await page.waitForTimeout(400);

    // Menu should be in a valid state (open)
    const ariaExpanded = await hamburger.getAttribute('aria-expanded');
    expect(ariaExpanded).toBe('true');

    // Should still be functional
    await hamburger.click();
    await page.waitForTimeout(400);
    const ariaExpandedAfter = await hamburger.getAttribute('aria-expanded');
    expect(ariaExpandedAfter).toBe('false');
  });
});
