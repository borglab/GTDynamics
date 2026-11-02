/** Typeset report TeX with pinned MathJax; emit standalone HTML with inline SVG. */
import { readFileSync, mkdtempSync, rmSync } from "node:fs";
import { execFileSync } from "node:child_process";
import { createRequire } from "node:module";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { fileURLToPath } from "node:url";

const require = createRequire(import.meta.url);
const fgzCli = require.resolve("fgz/packages/fgz-cli/dist/fgz2svg.js");
function diagram(name) {
  const directory = mkdtempSync(join(tmpdir(), "gtd-fgz-"));
  try {
    const source = fileURLToPath(new URL(`figures/${name}.fgz`, import.meta.url));
    const output = join(directory, `${name}.svg`);
    execFileSync(process.execPath, [fgzCli, source, "-o", output], {stdio: ["ignore", "pipe", "pipe"]});
    return `data:image/svg+xml;base64,${readFileSync(output).toString("base64")}`;
  } finally { rmSync(directory, {recursive: true, force: true}); }
}
const input = readFileSync(0, "utf8")
  .replace("__SUPPORT_GRAPH__", diagram("support"))
  .replace("__LANDMARK_GRAPH__", diagram("stance-landmarks"));
import { mathjax } from "mathjax-full/js/mathjax.js";
import { TeX } from "mathjax-full/js/input/tex.js";
import { SVG } from "mathjax-full/js/output/svg.js";
import { liteAdaptor } from "mathjax-full/js/adaptors/liteAdaptor.js";
import { RegisterHTMLHandler } from "mathjax-full/js/handlers/html.js";
import "mathjax-full/js/input/tex/ams/AmsConfiguration.js";
import "mathjax-full/js/input/tex/boldsymbol/BoldsymbolConfiguration.js";

const adaptor = liteAdaptor();
RegisterHTMLHandler(adaptor);
const document = mathjax.document(input, {
  InputJax: new TeX({ packages: ["base", "ams", "boldsymbol"], tags: "ams" }),
  OutputJax: new SVG({ fontCache: "local" }),
});
document.render();
const html = adaptor.outerHTML(adaptor.root(document.document));
const errors = html.match(/<g[^>]*data-mml-node="merror"[\s\S]{0,1000}/g);
if (errors) throw new Error("Invalid report TeX: " + errors.join("; "));
process.stdout.write("<!doctype html>\n" + html);
