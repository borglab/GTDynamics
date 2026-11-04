/** Typeset trusted, checked-in TeX at build time; no browser scripts or fonts. */
import { readFile } from 'node:fs/promises';
import { mathjax } from 'mathjax-full/js/mathjax.js';
import { TeX } from 'mathjax-full/js/input/tex.js';
import { SVG } from 'mathjax-full/js/output/svg.js';
import { liteAdaptor } from 'mathjax-full/js/adaptors/liteAdaptor.js';
import { RegisterHTMLHandler } from 'mathjax-full/js/handlers/html.js';
import 'mathjax-full/js/input/tex/ams/AmsConfiguration.js';
import 'mathjax-full/js/input/tex/boldsymbol/BoldsymbolConfiguration.js';

const adaptor = liteAdaptor();
RegisterHTMLHandler(adaptor);

export function typesetMath(source) {
  const document = mathjax.document(source, {
    InputJax: new TeX({ packages: ['base', 'ams', 'boldsymbol'] }),
    // Each equation owns its paths: no external fonts or cross-component IDs.
    OutputJax: new SVG({ fontCache: 'none' }),
  });
  document.render();
  const html = adaptor.innerHTML(adaptor.body(document.document));
  if (html.includes('data-mml-node="merror"')) throw new Error('Invalid paper TeX');
  return html;
}

export default function paperMath() {
  return {
    name: 'paper-math',
    enforce: 'pre',
    async load(id) {
      if (!id.endsWith('.math.html?math')) return null;
      const path = id.slice(0, -'?math'.length);
      this.addWatchFile(path);
      return `export default ${JSON.stringify(typesetMath(await readFile(path, 'utf8')))};`;
    },
  };
}
