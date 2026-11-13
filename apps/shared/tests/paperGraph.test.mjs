import test from 'node:test';
import assert from 'node:assert/strict';
import { graphSlices, paperGraph } from '../paper/graph.js';

test('explanatory graph selections stay inside short and endpoint recordings', () => {
  assert.deepEqual(graphSlices(0, 1), [0]);
  assert.deepEqual(graphSlices(0, 2), [0, 1]);
  assert.deepEqual(graphSlices(0, 715), [0, 1, 2]);
  assert.deepEqual(graphSlices(714, 715), [712, 713, 714]);
});

test('every schematic edge has a variable and clickable slices match the demo', () => {
  for (const kind of ['labels', 'initialization', 'smoothing', 'dynamics', 'kinodynamics', 'planning', 'scate', 'scate-reactive']) {
    for (const indices of [[0], [0, 1], [17, 18, 19]]) {
      const graph = paperGraph(kind, indices);
      const ids = new Set(graph.nodes.map(node => node.id));
      assert.equal(ids.size, graph.nodes.length);
      for (const factor of graph.factors) {
        for (const key of factor.keys) assert(ids.has(key), `${kind}: ${factor.title} → ${key}`);
      }
      for (const node of graph.nodes) {
        if (node.slice !== undefined) assert(indices.includes(node.slice));
      }
    }
  }
});

test('discrete graph shows per-foot displacement and duration scopes and four-key transitions', () => {
  const { factors } = paperGraph('labels', [0, 1, 2]);
  for (const factor of factors.filter(f => f.code === 'D')) {
    assert.equal(factor.keys.length, 2);
    assert.equal(factor.keys[0][0], factor.keys[1][0]);
  }
  assert(factors.filter(f => f.code === 'A/E').every(f => f.keys.length === 4));
  assert(factors.filter(f => f.code === 'ψ').every(f => f.keys.length === 3));
  assert(factors.filter(f => f.code === 'V').every(f => f.keys.length === 1));
});

test('smoothing diagram gives each IMU factor two poses, two velocities and shared bias', () => {
  const { factors } = paperGraph('smoothing', [0, 1, 2]);
  assert.deepEqual(factors.filter(f => f.code === 'I').map(f => f.keys), [
    ['z0', 'v0', 'z1', 'v1', 'b'], ['z1', 'v1', 'z2', 'v2', 'b'],
  ]);
});

test('coupled diagram separates free motion, acceleration, wrench and shared variables', () => {
  const { nodes, factors, caption } = paperGraph('kinodynamics', [0, 1, 2]);
  assert(nodes.some(node => node.id === 'b'));
  assert(nodes.some(node => node.id === 'c'));
  assert.equal(factors.filter(factor => factor.code === 'T').length, 2);
  assert(factors.filter(factor => factor.code === 'T').every(factor => factor.keys.length === 4));
  assert(factors.filter(factor => factor.code === 'I').every(factor => factor.keys.includes('b')));
  assert.match(caption, /hard constraints/);
  assert.match(caption, /free link COM poses/);
});

test('SCATE separates the obstacle observation from fixed-map collision factors', () => {
  const { factors } = paperGraph('scate', [4, 5, 6]);
  assert(factors.filter(f => f.code === 'Ml').every(f => f.keys.length === 2 && f.keys.includes('l')));
  assert(factors.filter(f => f.code === 'D').every(f => f.keys.length === 5));
  assert(factors.filter(f => f.code === 'O/S').every(f => f.keys.length === 4 && !f.keys.includes('l')));
});

test('reactive SCATE puts the base-disturbance factor on one state and its control', () => {
  const { factors } = paperGraph('scate-reactive', [4, 5, 6]);
  const base = factors.filter(f => f.code === 'B');
  assert.equal(base.length, 2);
  assert(base.every(f => f.keys.length === 3 && f.keys.some(k => k.startsWith('u'))));
  assert(factors.filter(f => f.code === 'O/S').every(f => !f.keys.some(k => k.startsWith('l'))));
  assert(factors.filter(f => f.code === 'Mℓ').every(f => f.keys.length === 1 && f.keys[0].startsWith('l')));
});
