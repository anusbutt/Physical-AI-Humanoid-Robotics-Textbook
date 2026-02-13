import { defineConfig, devices } from '@playwright/test';

/**
 * Playwright E2E test configuration for the RAG Chatbot.
 *
 * Run against production:
 *   npx playwright test
 *
 * Run against local dev:
 *   npx playwright test --config=playwright.config.ts
 */
export default defineConfig({
  testDir: './tests/e2e',
  fullyParallel: false, // Chat tests are sequential (multi-turn)
  forbidOnly: !!process.env.CI,
  retries: process.env.CI ? 1 : 0,
  workers: 1,
  reporter: 'html',

  use: {
    baseURL: process.env.E2E_BASE_URL || 'https://anusbutt.github.io/hackathon-phase-01/',
    trace: 'on-first-retry',
    screenshot: 'only-on-failure',
    actionTimeout: 10_000,
  },

  timeout: 60_000, // 60s per test (LLM responses can be slow)

  projects: [
    {
      name: 'chromium',
      use: { ...devices['Desktop Chrome'] },
    },
  ],
});
