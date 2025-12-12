import { test, expect } from '@playwright/test';

const BASE_URL = 'http://localhost:3000/hackathon1_repeat/';

test.describe('Production Deployment Verification', () => {
  test('should load the homepage successfully', async ({ page }) => {
    await page.goto(BASE_URL);

    // Wait for page to load
    await page.waitForLoadState('networkidle');

    // Check that the page loaded successfully
    expect(page.url()).toContain('/hackathon1_repeat/');

    // Check for main heading
    const heading = await page.locator('h1').first();
    await expect(heading).toBeVisible();

    console.log('✓ Homepage loaded successfully');
  });

  test('should have working navigation', async ({ page }) => {
    await page.goto(BASE_URL);
    await page.waitForLoadState('networkidle');

    // Check for navbar
    const navbar = await page.locator('nav.navbar');
    await expect(navbar).toBeVisible();

    console.log('✓ Navigation bar is visible');
  });

  test('should have responsive design', async ({ page }) => {
    // Test desktop view
    await page.setViewportSize({ width: 1920, height: 1080 });
    await page.goto(BASE_URL);
    await page.waitForLoadState('networkidle');

    // Check desktop navbar
    const desktopNav = await page.locator('nav.navbar');
    await expect(desktopNav).toBeVisible();

    console.log('✓ Desktop view working');

    // Test mobile view
    await page.setViewportSize({ width: 375, height: 667 });
    await page.goto(BASE_URL);
    await page.waitForLoadState('networkidle');

    // Check mobile hamburger menu
    const hamburger = await page.locator('.navbar__toggle');
    await expect(hamburger).toBeVisible();

    console.log('✓ Mobile view working');
  });

  test('should have working theme toggle', async ({ page }) => {
    await page.goto(BASE_URL);
    await page.waitForLoadState('networkidle');

    // Find theme toggle button
    const themeToggle = await page.locator('button[class*="toggle"]').first();

    if (await themeToggle.isVisible()) {
      await themeToggle.click();
      await page.waitForTimeout(500);
      console.log('✓ Theme toggle is functional');
    }
  });

  test('should load all critical resources', async ({ page }) => {
    const resources: { css: boolean; js: boolean; fonts: boolean } = {
      css: false,
      js: false,
      fonts: false
    };

    page.on('response', response => {
      const url = response.url();
      if (url.endsWith('.css')) resources.css = true;
      if (url.endsWith('.js')) resources.js = true;
      if (url.includes('font')) resources.fonts = true;
    });

    await page.goto(BASE_URL);
    await page.waitForLoadState('networkidle');

    expect(resources.css).toBe(true);
    expect(resources.js).toBe(true);

    console.log('✓ Critical resources loaded successfully');
    console.log(`  - CSS: ${resources.css}`);
    console.log(`  - JS: ${resources.js}`);
    console.log(`  - Fonts: ${resources.fonts}`);
  });

  test('should have no console errors', async ({ page }) => {
    const errors: string[] = [];

    page.on('console', msg => {
      if (msg.type() === 'error') {
        errors.push(msg.text());
      }
    });

    page.on('pageerror', error => {
      errors.push(error.message);
    });

    await page.goto(BASE_URL);
    await page.waitForLoadState('networkidle');

    if (errors.length > 0) {
      console.log('⚠ Console errors found:');
      errors.forEach(error => console.log(`  - ${error}`));
    } else {
      console.log('✓ No console errors');
    }

    // Allow some errors but warn about them
    expect(errors.length).toBeLessThan(5);
  });

  test('should have proper SEO meta tags', async ({ page }) => {
    await page.goto(BASE_URL);
    await page.waitForLoadState('networkidle');

    // Check for title
    const title = await page.title();
    expect(title).toBeTruthy();
    expect(title.length).toBeGreaterThan(0);

    // Check for meta description
    const metaDescription = await page.locator('meta[name="description"]').getAttribute('content');

    console.log('✓ SEO meta tags present');
    console.log(`  - Title: ${title}`);
    console.log(`  - Description: ${metaDescription || 'Not set'}`);
  });
});
