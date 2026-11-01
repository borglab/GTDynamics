import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {decodeStates,validateRecording} from './recording.js';
export function readMetadata(directory,name='recording.json.gz'){
  const bytes=readFileSync(new URL(name,directory));
  return JSON.parse(name.endsWith('.gz')?gunzipSync(bytes):bytes);
}
export function readRecording(directory,packed=false){
  const record=readMetadata(directory,typeof packed==='string'?`${packed}.json.gz`:packed?'smoothing.json.gz':'recording.json.gz');
  const bytes=gunzipSync(readFileSync(new URL(record.storage.states,directory)));
  const floats=decodeStates(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength),record.storage);
  validateRecording(record,floats);return {record,floats};
}
export function poseBlock(data,state){return data.floats.subarray(state.poseOffset,state.poseOffset+state.frames*data.record.bodies*7);}
