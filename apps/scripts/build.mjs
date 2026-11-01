import { access, cp, mkdir, writeFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { resolve } from 'node:path';
import { build } from 'vite';
import apps from '../registry.json' with { type: 'json' };

const root = fileURLToPath(new URL('../', import.meta.url));
try {
  await Promise.all([
    'motion-planning/public/assets/manifest.json',
    'motion-planning/public/assets/franka_emika_panda/scene.xml',
    'motion-planning/public/recording.json',
    'contact-initialization/public/assets/manifest.json',
    'contact-initialization/public/assets/unitree_g1/g1.xml',
  ].map(path => access(resolve(root, path))));
} catch (error) {
  throw new Error('App assets are missing. Run npm run models before building.', { cause: error });
}
// Relative assets keep the same artifact usable at /, /GTDynamics/, or a custom domain.
await build({ root, base: './' });
for (const app of apps) {
  const appRoot = resolve(root, app.id);
  await build({ root: appRoot, base: './' });
  // Vite materializes the bootstrap symlinks; the Pages artifact contains real files.
  await cp(resolve(appRoot, 'dist'), resolve(root, 'dist', app.id), {
    recursive: true, dereference: true,
  });
}
await mkdir(resolve(root, 'dist'), { recursive: true });
await writeFile(resolve(root, 'dist/.nojekyll'), '');
