import './theme.css';

export const THEME_KEY = 'gtdynamics-theme';
export const THEME_EVENT = 'gtdynamics-theme-change';
export const isTheme = value => value === 'light' || value === 'dark';
export const getTheme = () => document.documentElement.dataset.theme || 'light';

/** Apply a shared preference without recreating viewers or losing playback state. */
export function setTheme(theme, { persist = true } = {}) {
  if (!isTheme(theme)) return;
  if (persist) {
    try { localStorage.setItem(THEME_KEY, theme); } catch { /* Storage may be disabled. */ }
  }
  const changed = document.documentElement.dataset.theme !== theme;
  document.documentElement.dataset.theme = theme;
  if (changed) window.dispatchEvent(new CustomEvent(THEME_EVENT, { detail: theme }));
  // The shell and its app share an origin (also through the development proxy).
  for (const frame of document.querySelectorAll('iframe')) {
    frame.contentWindow?.postMessage({ type: THEME_EVENT, theme }, location.origin);
  }
}

export function onThemeChange(callback) {
  window.addEventListener(THEME_EVENT, callback);
  return () => window.removeEventListener(THEME_EVENT, callback);
}

/** Canvas and Plotly need concrete colors, not CSS variable strings. */
export function themeColor(role) {
  return getComputedStyle(document.documentElement).getPropertyValue(`--theme-${role}`).trim();
}

let initial = 'light';
try {
  const saved = localStorage.getItem(THEME_KEY);
  if (isTheme(saved)) initial = saved;
} catch { /* The default remains usable without storage. */ }
setTheme(initial, { persist: false });
window.addEventListener('storage', event => {
  if (event.key === THEME_KEY || event.key === null) {
    setTheme(isTheme(event.newValue) ? event.newValue : 'light', { persist: false });
  }
});
window.addEventListener('message', event => {
  if (event.origin !== location.origin || event.source !== window.parent || window.parent === window) return;
  if (event.data?.type === THEME_EVENT) setTheme(event.data.theme, { persist: false });
});
