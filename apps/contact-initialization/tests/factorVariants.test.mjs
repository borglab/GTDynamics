import assert from 'node:assert/strict';
import test from 'node:test';
import {bestTranslationRow, factorVariants} from '../src/factorVariants.js';

const errors = [1.7, 1.6, 0.2, 0.16, 1.7, 1.6, 0.15, 0.1];
const rows = factorVariants.map((choice,i)=>({
  ...choice, run:{evaluation:{positionRmseMeters:errors[i]}},
}));

test('automatic selection cannot enable a disabled height prior',()=>{
  assert.equal(bestTranslationRow(rows,false).id,'both');
});

test('automatic selection preserves an explicitly enabled height prior',()=>{
  assert.equal(bestTranslationRow(rows,true).id,'all');
  const otherModel = rows.map(row=>({...row,run:{evaluation:{positionRmseMeters:row.id==='none'?0:1}}}));
  assert.equal(bestTranslationRow(otherModel,true).ground,true);
});
