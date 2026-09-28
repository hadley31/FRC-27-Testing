/**
 * Installs a headless DOM for the tests that need one, once per process.
 *
 * Bun runs every test file in the same process, and Happy DOM refuses a second global registration — so
 * this has to be shared rather than repeated in each file.
 */

import { GlobalRegistrator } from '@happy-dom/global-registrator';

if (typeof globalThis.document === 'undefined') {
  GlobalRegistrator.register();
}

// Nothing in these tests changes size, and the charts observe their host for resizes.
globalThis.ResizeObserver ??= class {
  observe() {}
  unobserve() {}
  disconnect() {}
} as unknown as typeof ResizeObserver;
