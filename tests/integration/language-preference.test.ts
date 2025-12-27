/**
 * Language Preference Persistence Integration Tests
 * Phase 5: Translation Feature - T029
 *
 * Tests that language preference is correctly persisted in localStorage
 * and restored on page reload.
 */

import { test, expect, Page } from '@playwright/test';

const STORAGE_KEY = 'hackathon1_language_preference';

/**
 * Helper to get localStorage value
 */
async function getStoredPreference(page: Page): Promise<string | null> {
  return page.evaluate((key) => localStorage.getItem(key), STORAGE_KEY);
}

/**
 * Helper to set localStorage value
 */
async function setStoredPreference(page: Page, value: string): Promise<void> {
  await page.evaluate(
    ([key, val]) => localStorage.setItem(key, val),
    [STORAGE_KEY, value]
  );
}

/**
 * Helper to clear localStorage
 */
async function clearStoredPreference(page: Page): Promise<void> {
  await page.evaluate((key) => localStorage.removeItem(key), STORAGE_KEY);
}

/**
 * Helper to switch language using the toggle
 */
async function switchLanguage(page: Page, targetLang: 'en' | 'ur'): Promise<void> {
  // Click the language toggle button
  const toggle = page.getByRole('button', { name: /language|زبان/i });
  await toggle.click();

  // Select the target language
  const langLabel = targetLang === 'ur' ? 'اردو' : 'English';
  const option = page.getByRole('menuitem', { name: new RegExp(langLabel, 'i') });
  await option.click();

  // Wait for language change to apply
  await page.waitForTimeout(200);
}

test.describe('Language Preference Persistence', () => {
  test.beforeEach(async ({ page }) => {
    // Clear any existing preference
    await page.goto('/');
    await clearStoredPreference(page);
  });

  test('stores language preference when changed', async ({ page }) => {
    await page.goto('/docs/intro');

    // Switch to Urdu
    await switchLanguage(page, 'ur');

    // Check localStorage
    const stored = await getStoredPreference(page);
    expect(stored).not.toBeNull();

    const preference = JSON.parse(stored!);
    expect(preference.language).toBe('ur');
    expect(preference.updatedAt).toBeDefined();
  });

  test('restores language preference on page reload', async ({ page }) => {
    await page.goto('/docs/intro');

    // Switch to Urdu
    await switchLanguage(page, 'ur');

    // Reload the page
    await page.reload();

    // Wait for language context to initialize
    await page.waitForTimeout(500);

    // Check that Urdu is still selected
    const toggle = page.getByRole('button', { name: /language|زبان/i });
    await expect(toggle).toContainText(/اردو|UR/i);
  });

  test('restores language preference after browser restart (new session)', async ({
    browser,
  }) => {
    // First session - set preference
    const context1 = await browser.newContext();
    const page1 = await context1.newPage();

    await page1.goto('/docs/intro');
    await switchLanguage(page1, 'ur');

    // Get the stored value
    const stored = await getStoredPreference(page1);
    await context1.close();

    // Second session - restore preference
    const context2 = await browser.newContext({
      storageState: {
        cookies: [],
        origins: [
          {
            origin: page1.url().split('/').slice(0, 3).join('/'),
            localStorage: [
              { name: STORAGE_KEY, value: stored! },
            ],
          },
        ],
      },
    });
    const page2 = await context2.newPage();

    await page2.goto('/docs/intro');
    await page2.waitForTimeout(500);

    // Check that Urdu is restored
    const toggle = page2.getByRole('button', { name: /language|زبان/i });
    await expect(toggle).toContainText(/اردو|UR/i);

    await context2.close();
  });

  test('defaults to English when no preference is stored', async ({ page }) => {
    await page.goto('/docs/intro');

    // Ensure no preference is stored
    await clearStoredPreference(page);
    await page.reload();
    await page.waitForTimeout(500);

    // Check that English is selected
    const toggle = page.getByRole('button', { name: /language|زبان/i });
    await expect(toggle).toContainText(/English|EN/i);
  });

  test('handles corrupted localStorage gracefully', async ({ page }) => {
    await page.goto('/docs/intro');

    // Set corrupted value
    await setStoredPreference(page, 'not-valid-json{{{');
    await page.reload();
    await page.waitForTimeout(500);

    // Should fall back to English
    const toggle = page.getByRole('button', { name: /language|زبان/i });
    await expect(toggle).toContainText(/English|EN/i);
  });

  test('handles invalid language code gracefully', async ({ page }) => {
    await page.goto('/docs/intro');

    // Set invalid language
    await setStoredPreference(
      page,
      JSON.stringify({ language: 'invalid', updatedAt: new Date().toISOString() })
    );
    await page.reload();
    await page.waitForTimeout(500);

    // Should fall back to English
    const toggle = page.getByRole('button', { name: /language|زبان/i });
    await expect(toggle).toContainText(/English|EN/i);
  });

  test('updates preference timestamp on each change', async ({ page }) => {
    await page.goto('/docs/intro');

    // First change
    await switchLanguage(page, 'ur');
    const stored1 = await getStoredPreference(page);
    const pref1 = JSON.parse(stored1!);

    // Wait a bit
    await page.waitForTimeout(100);

    // Second change
    await switchLanguage(page, 'en');
    const stored2 = await getStoredPreference(page);
    const pref2 = JSON.parse(stored2!);

    // Timestamps should be different
    expect(new Date(pref2.updatedAt).getTime()).toBeGreaterThan(
      new Date(pref1.updatedAt).getTime()
    );
  });

  test('preference persists across different pages', async ({ page }) => {
    // Set preference on one page
    await page.goto('/docs/intro');
    await switchLanguage(page, 'ur');

    // Navigate to different page
    await page.goto('/docs/module-1-ros2-fundamentals/chapter-1-publishers');
    await page.waitForTimeout(500);

    // Check preference is maintained
    const toggle = page.getByRole('button', { name: /language|زبان/i });
    await expect(toggle).toContainText(/اردو|UR/i);
  });
});

test.describe('Language Toggle Accessibility', () => {
  test('announces language change to screen readers', async ({ page }) => {
    await page.goto('/docs/intro');

    // Switch language
    await switchLanguage(page, 'ur');

    // Check for aria-live region
    const announcer = page.locator('#language-announcer');
    await expect(announcer).toHaveAttribute('aria-live', 'polite');
  });
});
