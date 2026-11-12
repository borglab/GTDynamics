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
  const labels='apps/contact-labels/public/';
  for(const name of ['snippets.json','model.json.gz','MODEL_LICENSE.txt','SSIO_ATTRIBUTION.txt'])add(labels+name);
  for(const snippet of json(`${labels}snippets.json`))add(`${labels}snippets/${snippet.id}.json.gz`);
  const kinematics='apps/kinematics-inspector/public/';
  add(`${kinematics}MODEL_LICENSE.txt`);add(`${kinematics}comparison.json`);
  for(const prior of ['none','upright','height','upright-height'])for(const imu of ['', '-imu'])for(const loop of ['', '-loop'])for(const stage of ['initial','final']){
    replay(`${kinematics}variants/${prior}${imu}${loop}/${stage}.json.gz`);
  }
  const interpolation='apps/interpolation/public/';
  for(const name of ['recording.json.gz','states.bin.gz','model.json.gz','MODEL_LICENSE.txt','SSIO_ATTRIBUTION.txt'])add(interpolation+name);
  for(const name of ['recording.json','architecture.html'])add(`apps/motion-planning/public/${name}`);
  for(const name of ['recording.json.gz','model.json.gz','MODEL_LICENSE.txt','SSIO_ATTRIBUTION.txt']){add(`apps/dynamics/public/${name}`);add(`apps/kinodynamics/public/${name}`);}
  for(const robot of ['ur5','ur10e','ur10e-reactive'])for(const name of ['replay.json','meshes.json'])add(`apps/scate/${robot}/public/${name}`);
  for(const name of ['meshes-ur5.json','robots/ur10e.json','robots/ur5.json'])add(`apps/scate/ur10e-reactive/public/${name}`);
  const published=execFileSync('git',['ls-files','--cached','--others','--exclude-standard','-z','apps'],{cwd:root,encoding:'utf8'})
    .split('\0').filter(path=>/^apps\/(?:[^/]+\/)+public\//.test(path)&&existsSync(resolve(root,path)));
  assert.deepEqual(published.filter(path=>!used.has(path)),[],'Unused public assets would be uploaded');
  assert.deepEqual([...used].filter(path=>!published.includes(path)),[],'Replay depends on an untracked or ignored asset');
  const large=published.filter(path=>readFileSync(resolve(root,path)).byteLength>1024*1024);
  const attributes=execFileSync('git',['check-attr','-z','--stdin','filter'],{cwd:root,input:large.join('\0')+'\0',encoding:'utf8'}).split('\0');
  for(let i=0;i<attributes.length-1;i+=3)assert.equal(attributes[i+2],'lfs',`Large app asset must use LFS: ${attributes[i]}`);
});
