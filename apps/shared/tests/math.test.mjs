import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { typesetMath } from '../paper/mathPlugin.mjs';

for (const [name, minimum] of [['contact-labels', 10], ['contact-pose', 4], ['kinematics', 6], ['kinodynamics', 8]]) {
  test(`${name}: equations build to self-contained SVG without TeX errors`, () => {
    const source = readFileSync(new URL(`../paper/equations/${name}.math.html`, import.meta.url), 'utf8');
    const html = typesetMath(source);
    assert((html.match(/<mjx-container/g) || []).length >= minimum);
    assert(!/data-mml-node="merror"|<script|<use|<link/.test(html));
    assert(!/\\\[|\\\]/.test(html), 'No untypeset display delimiters');
    assert(!/https?:\/\//.test(html.replaceAll('http://www.w3.org/2000/svg', '')));
  });
}
test('invalid TeX fails the build instead of publishing broken equations', () => {
  assert.throws(() => typesetMath(String.raw`\[\undefinedPaperCommand{x}\]`), /Invalid paper TeX/);
});
