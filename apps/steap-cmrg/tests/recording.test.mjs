import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {gunzipSync} from 'node:zlib';
import {frameAtTime,segmentAtFrame,validateRecording} from '../src/recording.js';

const publicDir=fileURLToPath(new URL('../public/',import.meta.url));
const read=name=>readFileSync(publicDir+name);
const json=name=>JSON.parse(name.endsWith('.gz')?gunzipSync(read(name)):read(name));
const presets=json('presets.json');
const model=json('model.json.gz');

test('every listed preset is a valid recording of the shared cell model',()=>{
  assert(presets.length>0);
  for(const preset of presets){
    const record=json(preset.file);
    assert.equal(validateRecording(record,model),record);
    assert.equal(record.bodies.static.length,model.nbody);
  }
});

test('segments tile the frames and follow the build order',()=>{
  for(const preset of presets){
    const record=json(preset.file);
    let next=0;
    for(const segment of record.segments){
      assert.equal(segment.frameStart,next,`${segment.key} starts where the previous ended`);
      next+=segment.frameCount;
      assert(segment.duration>0);
      assert(Number.isFinite(segment.graphError));
      assert.equal(segment.toolError.length,2);
    }
    assert.equal(next,record.frames.time.length);
    const members=record.structure.members.filter(member=>!member.fixed).map(member=>member.name);
    const order=[...new Set(record.segments.map(segment=>segment.member))];
    assert.deepEqual(order,members,'members are built in specification order');
  }
});

test('every maneuver replans from each support state and obeys physical limits',()=>{
  for(const preset of presets){
    const record=json(preset.file);
    for(const segment of record.segments){
      assert.equal(segment.replans.length,segment.planned.length);
      assert(segment.replans.every(replan=>replan.length===segment.planned.length));
      assert(segment.clearance.floor>0,`${segment.key} stays above the floor`);
    }
  }
});

test('each nail appears on the last frame of its firing maneuver',()=>{
  for(const preset of presets){
    const record=json(preset.file);
    const firing=record.segments.filter(segment=>segment.events.some(event=>event.type==='fire_nail'));
    assert.equal(record.nails.length,firing.length);
    const frames=firing.map(segment=>segment.frameStart+segment.frameCount-1).sort((a,b)=>a-b);
    assert.deepEqual(record.nails.map(nail=>nail.frame).sort((a,b)=>a-b),frames);
    for(const nail of record.nails) assert(Math.abs(Math.hypot(...nail.direction)-1)<1e-3);
  }
});

test('frame lookup is by time and segment lookup by frame',()=>{
  const times=[0,1,1,2,5];
  assert.equal(frameAtTime(times,-1),0);
  assert.equal(frameAtTime(times,1.5),2);
  assert.equal(frameAtTime(times,9),4);
  const segments=[{frameStart:0},{frameStart:3},{frameStart:7}];
  assert.equal(segmentAtFrame(segments,0),0);
  assert.equal(segmentAtFrame(segments,6),1);
  assert.equal(segmentAtFrame(segments,7),2);
});
