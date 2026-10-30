import './style.css';
import apps from './registry.json';

const tabs = document.querySelector('.tabs');
const overview = document.querySelector('#overview-panel');
const panel = document.querySelector('#app-panel');
const appList = document.querySelector('#app-list');
let selected;

for (const [index, app] of apps.entries()) {
  const tab = document.createElement('a');
  tab.className = 'tab';
  tab.id = `tab-${app.id}`;
  tab.href = `#${app.id}`;
  tab.role = 'tab';
  tab.setAttribute('aria-controls', 'app-panel');
  tab.textContent = app.title;
  tabs.append(tab);

  const link = document.createElement('a');
  link.className = 'app-link';
  link.href = `#${app.id}`;
  const number = document.createElement('span');
  number.className = 'app-number';
  number.textContent = String(index + 1).padStart(2, '0');
  const copy = document.createElement('div');
  const title = document.createElement('h3');
  title.textContent = app.title;
  const description = document.createElement('p');
  description.textContent = app.description;
  copy.append(title, description);
  const tag = document.createElement('span');
  tag.className = 'app-tag';
  tag.textContent = app.tag;
  const arrow = document.createElement('span');
  arrow.className = 'app-arrow';
  arrow.setAttribute('aria-hidden', 'true');
  arrow.textContent = '↗';
  link.append(number, copy, tag, arrow);
  appList.append(link);
}

function selectTab() {
  const id = location.hash.slice(1);
  const app = apps.find(app => app.id === id);
  const active = app?.id || 'overview';
  if (selected === active) return;
  selected = active;
  for (const tab of tabs.children) {
    const isSelected = tab.id === `tab-${active}`;
    tab.setAttribute('aria-selected', String(isSelected));
    tab.tabIndex = isSelected ? 0 : -1;
  }
  overview.hidden = Boolean(app);
  panel.hidden = !app;
  panel.replaceChildren();
  document.title = `${app?.title || 'Overview'} · GTDynamics`;
  if (app) {
    panel.setAttribute('aria-labelledby', `tab-${app.id}`);
    const frame = document.createElement('iframe');
    frame.title = `${app.title} · GTDynamics`;
    // Resolve from the document rather than the bundle's hashed assets/ directory.
    frame.src = new URL(`./${app.id}/`, location.href).href;
    panel.append(frame);
  }
  window.scrollTo(0, 0);
}

tabs.addEventListener('keydown', event => {
  const items = [...tabs.children];
  let index = items.indexOf(document.activeElement);
  if (event.key === 'ArrowRight') index = (index + 1) % items.length;
  else if (event.key === 'ArrowLeft') index = (index + items.length - 1) % items.length;
  else if (event.key === 'Home') index = 0;
  else if (event.key === 'End') index = items.length - 1;
  else return;
  event.preventDefault();
  items[index].focus();
  items[index].click();
});
window.addEventListener('hashchange', selectTab);
selectTab();
