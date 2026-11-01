import test from 'node:test';
import assert from 'node:assert/strict';
import {execFileSync} from 'node:child_process';
import {existsSync,readFileSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {gunzipSync} from 'node:zlib';

const root=fileURLToPath(new URL('../../../',import.meta.url));
const json=path=>JSON.parse(readFileSync(resolve(root,path)));

test('every published, checked-in app asset has a replay or attribution consumer',()=>{
  const used=new Set();
  const add=path=>{path=path.replaceAll('\\','/');assert(existsSync(resolve(root,path)),`Missing app asset: ${path}`);used.add(path);};
  const replay=path=>{
    add(path);
    const record=JSON.parse(gunzipSync(readFileSync(resolve(root,path))));
    for(const name of [record.storage.states,record.storage.model])add(resolve(root,dirname(path),name).slice(root.length));
    for(const credit of record.motionSource.licenseNotes)add(resolve(root,dirname(path),credit.localNotice).slice(root.length));
  };
  const contact='apps/contact-initialization/public/';
  add(`${contact}sequences.json`);
  for(const sequence of json(`${contact}sequences.json`).sequences){add(contact+sequence.data);add(contact+sequence.recording);}
  for(const name of ['G1_MOVES_ATTRIBUTION.txt','THIRD_PARTY_LICENSES.txt'])add(contact+name);
  const comparison='apps/contact-vs-ik/public/';
  add(`${comparison}snippets.json`);add(`${comparison}MODEL_LICENSE.txt`);
  for(const snippet of json(`${comparison}snippets.json`))for(const imu of ['', 'imu/']){
    const directory=`${comparison}snippets/${snippet.id}/${imu}`;
    replay(`${directory}recording.json.gz`);replay(`${directory}smoothing.json.gz`);
  }
  const kinematics='apps/kinematics-inspector/public/';
  add(`${kinematics}MODEL_LICENSE.txt`);
  for(const sigma of [50,20,5])for(const imu of ['', '-imu'])for(const loop of ['', '-loop'])for(const stage of ['initial','final']){
    replay(`${kinematics}variants/${sigma}${imu}${loop}/${stage}.json.gz`);
  }
  for(const name of ['recording.json','architecture.html'])add(`apps/motion-planning/public/${name}`);
  const published=execFileSync('git',['ls-files','--cached','--others','--exclude-standard','-z','apps'],{cwd:root,encoding:'utf8'})
    .split('\0').filter(path=>/^apps\/[^/]+\/public\//.test(path)&&existsSync(resolve(root,path)));
  assert.deepEqual(published.filter(path=>!used.has(path)),[],'Unused public assets would be uploaded');
  assert.deepEqual([...used].filter(path=>!published.includes(path)),[],'Replay depends on an untracked or ignored asset');
});
