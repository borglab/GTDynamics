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
  for (const kind of ['labels', 'initialization', 'smoothing', 'dynamics', 'planning']) {
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
