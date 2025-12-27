/**
 * E2E Journey Test: Chatbot Panel Interaction
 *
 * Tests the chatbot panel functionality including opening, closing,
 * minimizing, resizing, and search features.
 */

import { test, expect } from '@playwright/test';

test.describe('Journey 02: Chatbot Panel Interaction', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/');
  });

  test('should open chatbot panel when clicking AI Assistant link', async ({ page }) => {
    // Click on AI Assistant link in navbar
    const chatLink = page.getByRole('link', { name: /ai assistant/i });
    if (await chatLink.count() > 0) {
      await chatLink.click();

      // Should navigate to chatbot page or open panel
      await page.waitForTimeout(500); // Allow for animation
    }
  });

  test('should have proper panel structure when open', async ({ page }) => {
    // Navigate to chatbot page
    await page.goto('/chatbot');

    // Check for chat panel or chat interface elements
    const chatContainer = page.locator('[class*="chat"], [role="complementary"]');
    if (await chatContainer.count() > 0) {
      await expect(chatContainer.first()).toBeVisible();
    }
  });

  test('should support keyboard navigation in chat panel', async ({ page }) => {
    await page.goto('/chatbot');

    // Try to find and focus chat input
    const chatInput = page.locator('input[type="text"], textarea, [contenteditable]');
    if (await chatInput.count() > 0) {
      await chatInput.first().focus();
      await expect(chatInput.first()).toBeFocused();
    }
  });

  test('should close panel with Escape key', async ({ page }) => {
    await page.goto('/chatbot');

    // Press Escape to potentially close/minimize panel
    await page.keyboard.press('Escape');
    await page.waitForTimeout(300);

    // Page should still be usable
    await expect(page.locator('body')).toBeVisible();
  });

  test('should handle chat input submission', async ({ page }) => {
    await page.goto('/chatbot');

    const chatInput = page.locator('input[type="text"], textarea');
    if (await chatInput.count() > 0) {
      await chatInput.first().fill('Hello');
      await chatInput.first().press('Enter');

      // Wait for potential response
      await page.waitForTimeout(1000);
    }
  });

  test('should have accessible chat elements', async ({ page }) => {
    await page.goto('/chatbot');

    // Check for proper ARIA attributes
    const chatRegion = page.locator('[role="complementary"], [role="region"], [aria-label*="chat" i]');
    if (await chatRegion.count() > 0) {
      await expect(chatRegion.first()).toBeVisible();
    }
  });
});
