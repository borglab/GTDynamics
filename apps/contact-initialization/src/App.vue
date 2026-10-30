<script setup>
import { computed, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue';
import { ContactScene } from './scene.js';
import { quaternionToRotation, composePoses, inversePose, transformPoint } from './poseMath.js';

const data = ref(null), graph = ref(null), result = ref(null), error = ref('');
const sequences=ref([]),sequenceId=ref(''),recording=ref(null),nativeRun=ref(null);
const baseUrl=import.meta.env.BASE_URL;
const sceneHost = ref(null), graphHost = ref(null);
const stage = ref(2), model = ref('ground'), selected = ref(24), iteration = ref(0);
const showReference = ref(true), showGhosts = ref(true), playing = ref(false), loading = ref(false),replaying=ref(false);
let scene, clock, lastTick = 0, lastReplayTick=0,loadGeneration=0;
const steps = [
  {title:'Find the events', detail:'A stance begins. A stance ends. Keep those instants.'},
  {title:'Freeze the robot', detail:'Joint angles become rigid geometry, computed once with FK.'},
  {title:'Connect the contacts', detail:'Shared stance landmarks stitch the rigid frames into a walk.'},
];
const frames = computed(() => graph.value?.frames || []);
const contacts = computed(() => graph.value?.contacts || []);
const sequence=computed(()=>sequences.value.find(s=>s.id===sequenceId.value));
const benchmark=computed(()=>nativeRun.value?.benchmark);
const environment=computed(()=>nativeRun.value?.environment||{});
const iterationNumber=computed(()=>snapshot.value?.iteration||0);
const phaseTimings=computed(()=>[
  {id:'seed',title:'Contact seed',detail:'Rigid graph traversal'},
  {id:'graphBuild',title:'Build graph',detail:'GTSAM variables & factors'},
  {id:'optimization',title:'GTSAM LM',detail:'Sparse nonlinear solve'},
  {id:'total',title:'Initializer total',detail:'Seed + graph + solve'},
].map(p=>({...p,stats:benchmark.value?.phases?.[p.id]||{medianMs:0,minMs:0,p95Ms:0}})));
const modelTimings=computed(()=>['ground','patch','point'].map(id=>({id,title:id==='ground'?'Patch + ground':id==='patch'?'Contact-only patch':'One point',run:recording.value?.models[id]})).filter(p=>p.run));
const current = computed(() => frames.value[selected.value]);
const duration = computed(() => data.value?.samples.at(-1)?.t || 8);
const history = computed(() => result.value?.history || []);
const snapshot = computed(() => history.value[iteration.value] || result.value);
const displayPoses = computed(() => snapshot.value?.poses || result.value?.poses);
const landmarkPoints = computed(() => snapshot.value?.landmarks || result.value?.landmarks || []);
const referencePoses = computed(() => {
  if (!data.value?.reference?.poses || !graph.value) return [];
  const references = frames.value.map(f => {
    const p = data.value.reference.poses[f.sampleIndex];
    return {rotation:quaternionToRotation(p.q), translation:p.p};
  });
  if (!references.length) return [];
  // Reference enters only this evaluation/display path, after graph construction.
  const aligned=references.slice();
  const components=graph.value.components||[references.map((_,i)=>i)];
  for(const indices of components){
    const anchor=indices[0];
    const alignment=composePoses(graph.value.initialPoses[anchor],inversePose(references[anchor]));
    for(const index of indices)aligned[index]=composePoses(alignment,references[index]);
  }
  return aligned;
});
const comparison = computed(() => {
  const poses = displayPoses.value, truth = referencePoses.value;
  if (!poses?.length || truth.length !== poses.length) return {position:0, angle:0};
  let distance = 0, angle = 0;
  poses.forEach((pose, i) => {
    distance += pose.translation.reduce((sum, v, j) => sum + (v-truth[i].translation[j])**2, 0);
    const trace = pose.rotation.reduce((sum,v,j) => sum+v*truth[i].rotation[j],0);
    angle += Math.acos(Math.max(-1,Math.min(1,(trace-1)/2)))**2;
  });
  return {position:1000*Math.sqrt(distance/poses.length),angle:180/Math.PI*Math.sqrt(angle/poses.length)};
});
const activeIds = computed(() => [...new Set((current.value?.observations || []).map(o => o.contactId))]);
const activeContacts = computed(() => contacts.value.filter(c => activeIds.value.includes(c.id)));
const support = computed(() => activeContacts.value.map(c => c.foot === 'left' ? 'Left' : 'Right').join(' + ') || 'No support');
const componentCount = computed(() => graph.value?.anchorFrameIndices?.length || 1);
const frameX = i => 30 + i*49;
const graphWidth = computed(() => Math.max(620, frames.value.length*49 + 20));
const contactX = c => {
  const indices = frames.value.map((f,i) => f.observations.some(o => o.contactId === c.id) ? i : -1).filter(i=>i>=0);
  return indices.length ? frameX((indices[0]+indices.at(-1))/2) : 30;
};
const contactY = c => c.foot === 'left' ? 147 : 213;
const graphEdges = computed(() => frames.value.flatMap((f,i) => [...new Set(f.observations.map(o=>o.contactId))].map(id=> {
  const contact = contacts.value.find(c=>c.id===id);
  return {key:`${i}-${id}`,i,contact,x:frameX(i),cx:contactX(contact),cy:contactY(contact)};
})));
const ticks = computed(() => {
  const step=duration.value>15?5:duration.value>9?2:1;
  return Array.from({length:Math.floor(duration.value/step)+1},(_,i)=>i*step);
});
const timelineX = time => 68+time/duration.value*1100;
const loadPath = foot => {
  if (!data.value) return '';
  const lane = foot === 'left' ? 45 : 96;
  return data.value.samples.map((s,i)=>`${i?'L':'M'}${timelineX(s.t).toFixed(2)},${(lane+14-Math.min(1,(s.normalLoads?.[foot] || 0)/500)*22).toFixed(2)}`).join(' ');
};
const lossPath = computed(() => {
  if (!history.value.length) return '';
  const values = history.value.map(h=>Math.log10(Math.max(1e-12,h.objective ?? h.rms*h.rms)));
  const lo=Math.min(...values)-0.1, hi=Math.max(...values)+0.1;
  return values.map((v,i)=>`${i?'L':'M'}${8+i/Math.max(1,values.length-1)*270},${60-(v-lo)/(hi-lo)*50}`).join(' ');
});
const currentRms = computed(() => snapshot.value?.rms ?? snapshot.value?.residualRms ?? result.value?.residualRms ?? 0);
const observationCount = computed(() => frames.value.reduce((n,f)=>n+f.observations.length,0));
const unknownCount = computed(() => Math.max(0,frames.value.length-componentCount.value)*6 + (graph.value?.landmarks?.length || 0)*(model.value==='ground'?2:3));
const time = value => Number(value || 0).toFixed(2);
const contactLabel = c => `${c.foot === 'left' ? 'L' : 'R'}${String(c.id).replace(/[^0-9]/g,'') || contacts.value.indexOf(c)+1}`;
const sceneGeometry = computed(() => {
  if (!data.value || !graph.value || !result.value) return {};
  const points = landmarkPoints.value;
  const groundZ = graph.value.groundPlane?.height ?? graph.value.frames[0].observations.reduce((sum,o)=>sum+o.local[2],0)/Math.max(1,graph.value.frames[0].observations.length);
  const lift = p => ({...p,translation:[p.translation[0],p.translation[1],p.translation[2]-groundZ]});
  const sceneContacts = contacts.value.map(c => ({...c,points:c.landmarkIndices.map(i=>{
    if(stage.value!==0)return points[i];
    const frameIndex=frames.value.findIndex(f=>f.observations.some(o=>o.landmarkIndex===i));
    const observation=frames.value[frameIndex]?.observations.find(o=>o.landmarkIndex===i);
    return observation?transformPoint(referencePoses.value[frameIndex],observation.local):null;
  }).filter(Boolean).map(p=>[p[0],p[1],p[2]-groundZ])}));
  return {model:data.value.model,samples:data.value.samples,frames:frames.value,contacts:sceneContacts,
    poses:(stage.value===0 ? referencePoses.value : displayPoses.value).map(lift),referencePoses:referencePoses.value.map(lift),
    components:graph.value.components,floorConstrained:model.value==='ground'||stage.value===0,mode:stage.value===1?'frozen':'solved'};
});
function updateScene() {
  if (!scene || !data.value || !graph.value || !result.value) return;
  scene.update({...sceneGeometry.value, selectedIndex:selected.value,
    showReference:showReference.value&&stage.value===2,showGhosts:showGhosts.value&&stage.value!==1});
}
async function chooseModel() {
  replaying.value=false;playing.value=false;
  const run=recording.value?.models[model.value];
  if(!run)return;
  nativeRun.value=run;graph.value=run.graph;result.value=run.result;
  iteration.value=Math.max(0,run.result.history.length-1);
  selected.value=Math.min(selected.value,run.graph.frames.length-1);
  await nextTick();selectFrame(selected.value);updateScene();scene?.reset();
}
async function loadSequence() {
  const selectedSequence=sequence.value;
  if(!selectedSequence)return;
  const generation=++loadGeneration;
  loading.value=true;error.value='';playing.value=false;replaying.value=false;
  try {
    const responses=await Promise.all([selectedSequence.data,selectedSequence.recording].map(async path=>{
      const response=await fetch(`${import.meta.env.BASE_URL}${path}`);
      if(!response.ok)throw new Error(`Could not load ${path} (${response.status}).`);
      return response.json();
    }));
    if(generation!==loadGeneration)return;
    data.value=responses[0];recording.value=responses[1];
    if(recording.value.solver!=='C++ GTSAM Levenberg–Marquardt')throw new Error('This app requires a recorded C++ GTSAM solve.');
    selected.value=Math.min(24,recording.value.models[model.value].graph.frames.length-1);
    await chooseModel();await nextTick();
    if(!scene)scene=new ContactScene(sceneHost.value,{onSelect:selectFrame});
    updateScene();scene.reset();
  }catch(e){if(generation===loadGeneration)error.value=e.message;}
  finally{if(generation===loadGeneration)loading.value=false;}
}
function replayNative() {
  if(replaying.value){replaying.value=false;return;}
  playing.value=false;iteration.value=0;lastReplayTick=performance.now();replaying.value=true;
}
function selectFrame(index) {
  selected.value=Math.max(0,Math.min(frames.value.length-1,Number(index)));
  const el=graphHost.value;
  if(el) el.scrollTo({left:Math.max(0,frameX(selected.value)-el.clientWidth/2),behavior:'smooth'});
}
function exportInitialization() {
  const payload={schemaVersion:2,solver:recording.value.solver,sequenceId:sequenceId.value,source:sequence.value.sourceUrl,
    nativeSource:recording.value.source,benchmark:benchmark.value,
    model:model.value,assumptions:{stationaryMaterialPoints:true,groundPlane:graph.value.groundPlane||null,
      gauge:'First event pose fixed; reference root trajectory excluded.'},
    observability:graph.value.observability,contactRmsMeters:result.value.residualRms,
    eventPoses:frames.value.map((f,i)=>({sampleIndex:f.sampleIndex,time:f.time,...result.value.poses[i]})),
    contacts:contacts.value.map(c=>({id:c.id,foot:c.foot,start:c.start,end:c.end,points:c.landmarkIndices.map(i=>result.value.landmarks[i])}))};
  const url=URL.createObjectURL(new Blob([JSON.stringify(payload,null,2)],{type:'application/json'}));
  const a=document.createElement('a');a.href=url;a.download=`${sequenceId.value}-${model.value}-gtsam-initialization.json`;a.click();
  setTimeout(()=>URL.revokeObjectURL(url),1000);
}
function clickTimeline(event) {
  const rect=event.currentTarget.getBoundingClientRect();
  const t=((event.clientX-rect.left)/rect.width*1200-68)/1100*duration.value;
  let nearest=0;
  frames.value.forEach((f,i)=>{if(Math.abs(f.time-t)<Math.abs(frames.value[nearest].time-t))nearest=i;});
  selectFrame(nearest);
}
function animate(now) {
  if(playing.value && now-lastTick>420){
    selectFrame((selected.value+1)%frames.value.length);lastTick=now;
  }
  if(replaying.value&&now-lastReplayTick>280){
    if(iteration.value<history.value.length-1)iteration.value++;
    else replaying.value=false;
    lastReplayTick=now;
  }
  clock=requestAnimationFrame(animate);
}
watch([selected,iteration,stage,showReference,showGhosts],()=>nextTick(updateScene));
watch(model,chooseModel);
watch(sequenceId,loadSequence);
onMounted(async()=>{
  try {
    const response=await fetch(`${import.meta.env.BASE_URL}sequences.json`);
    if(!response.ok)throw new Error(`The sequence catalogue could not load (${response.status}).`);
    sequences.value=(await response.json()).sequences;
    if(!sequences.value.length)throw new Error('No recorded sequences are available.');
    sequenceId.value=sequences.value[0].id;
    clock=requestAnimationFrame(animate);
  }catch(e){error.value=e.message;}
});
onBeforeUnmount(()=>{cancelAnimationFrame(clock);scene?.dispose();});
</script>

<template>
  <main class="lab" :data-model="model" :data-sequence="sequenceId" :data-event="selected" :data-iteration="iterationNumber" :aria-busy="loading">
    <header class="intro">
      <div><p class="eyebrow"><span class="live-dot"></span> A WALKING INITIALIZER / UNITREE G1</p>
        <h1>Walking, reduced to <em>contact.</em></h1>
        <p class="intro-copy">A C++ GTSAM alternative to inverse-kinematics initialization. Freeze joint geometry, then solve for event poses and shared contacts.</p>
      </div>
      <div class="source-stamp"><span class="stamp-icon">C++</span><div><strong>GTSAM Levenberg–Marquardt</strong><span>Native solve · measured timings · recorded replay</span><a href="https://github.com/borglab/GTDynamics/pull/452" target="_blank" rel="noopener">C++ implementation & API ↗</a></div></div>
    </header>

    <div v-if="error" class="error-banner" role="alert">{{error}}</div>
    <div class="sequence-toolbar" v-if="sequences.length"><label for="sequence-select">MOTION SEQUENCE <select id="sequence-select" v-model="sequenceId" aria-label="Motion sequence" :disabled="loading"><option v-for="entry in sequences" :key="entry.id" :value="entry.id">{{entry.title}}</option></select></label><span>{{sequence?.description}}</span><a :href="sequence?.sourceUrl" target="_blank" rel="noopener">Original source ↗</a></div>
    <div v-if="loading" class="recording-loading" role="status">Loading recorded C++ solve…</div>
    <div v-if="!result && !error && !loading" class="loading">Loading the G1 sequence catalogue…</div>
    <template v-if="result">
      <nav class="steps" aria-label="Initialization stages">
        <button v-for="(step,i) in steps" :key="i" :class="{active:stage===i}" :aria-pressed="stage===i" @click="stage=i"><span class="step-number">0{{i+1}}</span><span><strong>{{step.title}}</strong><small>{{step.detail}}</small></span><span class="step-arrow">↗</span></button>
      </nav>

      <section class="workspace">
        <div class="scene-card">
          <div class="panel-top"><div><span class="section-label">{{stage===0?'THE MEASURED WALK':stage===1?'A RIGID SNAPSHOT':'THE RECONSTRUCTED WALK'}}</span><h2>{{stage===0?'Every stance has a beginning and an end.':stage===1?'29 joint angles become constants.':'One pose per event. One place per contact.'}}</h2></div><span class="status-tag" :class="{amber:model==='point'}">{{model==='ground'?'SOLE PATCH + LEVEL GROUND':model==='patch'?'SOLE PATCH · CONTACTS ONLY':'POINT MODEL · AMBIGUOUS'}}</span></div>
          <div class="scene-wrap"><div ref="sceneHost" class="scene" role="img" aria-label="Interactive 3D G1 robot with contact footprints and reconstructed event poses"></div>
            <div class="scene-readout"><span class="readout-label">EVENT {{String(selected).padStart(2,'0')}}</span><strong>{{time(current?.time)}} <small>s</small></strong><span>{{support}} support</span></div>
            <div class="scene-legend"><span><i class="dot left"></i>Left contact</span><span><i class="dot right"></i>Right contact</span><span v-if="showReference&&stage===2"><i class="line-key"></i>Reference</span></div>
            <div class="camera-controls"><button @click="scene?.setView('perspective')" title="Perspective view">3D</button><button @click="scene?.setView('side')">Side</button><button @click="scene?.setView('top')">Top</button><button @click="scene?.reset()" title="Reset camera" aria-label="Reset camera">⤢</button></div>
            <span class="orbit-hint">DRAG TO ORBIT · SCROLL TO ZOOM</span>
          </div>
          <div class="playback"><button class="play-button" :aria-label="playing?'Pause event playback':'Play event playback'" @click="playing=!playing">{{playing?'Ⅱ':'▶'}}</button><button class="skip-button" aria-label="Previous event" @click="selectFrame(selected-1)">‹</button><input aria-label="Contact event" type="range" min="0" :max="frames.length-1" :value="selected" @input="selectFrame($event.target.value)"/><button class="skip-button" aria-label="Next event" @click="selectFrame(selected+1)">›</button><span class="mono">{{selected+1}} / {{frames.length}}</span><div class="view-options"><label><input type="checkbox" v-model="showGhosts"/>Snapshots</label><label><input type="checkbox" v-model="showReference"/>Reference</label></div></div>
        </div>

        <aside class="graph-card">
          <div class="panel-top"><div><span class="section-label">THE FACTOR GRAPH</span><h2>Contacts are the connections.</h2></div><span class="mini-tag">SE(3) + ℝ³</span></div>
          <div class="graph-stats"><div><strong>{{frames.length}}</strong><span>event poses</span></div><span class="stats-plus">+</span><div><strong>{{contacts.length}}</strong><span>stance patches</span></div><span class="stats-equals">=</span><div><strong>{{unknownCount}}</strong><span>free scalars</span></div></div>
          <div class="graph-scroll" ref="graphHost"><svg :viewBox="`0 0 ${graphWidth} 260`" :width="graphWidth" height="260" role="img" aria-label="Bipartite graph of event poses and stance contact landmarks">
            <text x="12" y="18" class="svg-label">POSES / TIME →</text><text x="12" y="252" class="svg-label">CONTACT LANDMARK GROUPS / ONE PER STANCE</text>
            <g v-for="edge in graphEdges" :key="edge.key" :class="['edge',edge.contact.foot,{highlight:edge.i===selected}]"><path :d="`M${edge.x} 62 L${edge.cx} ${edge.cy}`"/><rect :x="(edge.x+edge.cx)/2-2.5" :y="(62+edge.cy)/2-2.5" width="5" height="5"/></g>
            <g v-for="(frame,i) in frames" :key="frame.id" class="pose-node" :class="{selected:i===selected,anchor:graph.anchorFrameIndices.includes(i)}" role="button" tabindex="0" :aria-label="`Select event ${i}, ${time(frame.time)} seconds`" @click="selectFrame(i)" @keydown.enter="selectFrame(i)" @keydown.space.prevent="selectFrame(i)"><circle :cx="frameX(i)" cy="49" r="14"/><text :x="frameX(i)" y="53">X<tspan font-size="8" dy="3">{{i}}</tspan></text><text :x="frameX(i)" y="83" class="node-time">{{time(frame.time)}}</text><path v-if="graph.anchorFrameIndices.includes(i)" :d="`M${frameX(i)-6} 29h12m-6 0v-7`" class="anchor-mark"/></g>
            <g v-for="contact in contacts" :key="contact.id" class="contact-node" :class="[contact.foot,{active:activeIds.includes(contact.id)}]"><rect :x="contactX(contact)-13" :y="contactY(contact)-10" width="26" height="20" rx="5"/><text :x="contactX(contact)" :y="contactY(contact)+4">{{contactLabel(contact)}}</text></g>
          </svg></div>
          <div class="graph-key"><span><i class="circle-key"></i>Pose variable</span><span><i class="square-key"></i>Contact factor</span><span class="anchor-key">{{componentCount===1?'X₀ fixed':`${componentCount} separate anchors`}}</span></div>
          <div class="equation"><span class="equation-main">Rᵢ pᵢⱼ + tᵢ − cⱼ = 0</span><p><b>pᵢⱼ</b> is known from forward kinematics.<br/><b>Xᵢ = (Rᵢ, tᵢ)</b> and <b>cⱼ</b> are the unknowns.</p></div>
          <div class="model-picker"><label for="contact-model">CONTACT GEOMETRY</label><select id="contact-model" v-model="model" aria-label="Contact geometry"><option value="ground">Sole patch + level ground</option><option value="patch">Sole patch · contacts only</option><option value="point">One foot point · ambiguous</option></select></div>
          <p class="model-note">{{model==='ground'?'Adds a level-floor assumption: all stance landmarks lie on one horizontal plane. Sole geometry sets the initial tilt; this extra constraint limits rocking-induced drift.':model==='patch'?'Three noncollinear material points constrain a rigid foot patch. G1 rocks its feet, so this pure contact model accumulates large attitude drift.':'One fixed point leaves three rotations free; two leave rotation about their connecting line. A low residual cannot certify the full pose.'}}</p>
        </aside>
      </section>

      <section class="timeline-card">
        <div class="timeline-heading"><div><span class="section-label">THE CONTACT CLOCK</span><h2>A frame at every touchdown and lift-off.</h2></div><span class="timeline-meta">{{data.samples.length}} joint samples → {{frames.length}} event frames <i>·</i> {{(data.samples.length/frames.length).toFixed(1)}}× fewer poses</span></div>
        <svg class="timeline" viewBox="0 0 1200 145" @click="clickTimeline" role="img" :aria-label="`Left and right stance intervals over ${duration.toFixed(1)} seconds; click to select the nearest event`">
          <g v-for="tick in ticks" :key="tick"><line :x1="timelineX(tick)" y1="18" :x2="timelineX(tick)" y2="122" class="timeline-grid"/><text :x="timelineX(tick)" y="140" text-anchor="middle" class="svg-label">{{tick}}s</text></g>
          <text x="8" y="48" class="timeline-side left-fill">LEFT</text><text x="8" y="99" class="timeline-side right-fill">RIGHT</text>
          <rect x="68" y="26" width="1100" height="36" rx="4" class="timeline-lane"/><rect x="68" y="77" width="1100" height="36" rx="4" class="timeline-lane"/>
          <g v-for="contact in contacts" :key="contact.id" :class="['stance',contact.foot,{active:activeIds.includes(contact.id)}]"><rect :x="timelineX(data.samples[contact.start]?.t)" :y="contact.foot==='left'?26:77" :width="Math.max(2,timelineX(data.samples[contact.end]?.t)-timelineX(data.samples[contact.start]?.t))" height="36" rx="3"/><text v-if="contact.end-contact.start>15" :x="timelineX((data.samples[contact.start]?.t+data.samples[contact.end]?.t)/2)" :y="contact.foot==='left'?47:98" text-anchor="middle">{{contactLabel(contact)}}</text></g>
          <path :d="loadPath('left')" class="load-trace left"/><path :d="loadPath('right')" class="load-trace right"/>
          <line v-for="frame in frames" :key="frame.id" :x1="timelineX(frame.time)" y1="18" :x2="timelineX(frame.time)" y2="120" class="event-tick"/>
          <line :x1="timelineX(current?.time)" y1="10" :x2="timelineX(current?.time)" y2="122" class="playhead"/><circle :cx="timelineX(current?.time)" cy="10" r="4" class="playhead-dot"/>
        </svg>
        <div class="timeline-caption"><span>Filled bars: stance episodes <i>·</i><template v-if="data.samples[0].normalLoads"> Trace: replayed normal load <i>·</i></template> Vertical ticks: frozen frames</span><span>{{data.samples[0].normalLoads?'Contact labels from replayed simulator forces.':'Contact labels from a height/speed heuristic, using the source root only for labeling.'}}</span></div>
        <div v-if="componentCount>1" class="component-notice">{{componentCount}} disconnected support components. Each has a separate pose anchor; the contact graph cannot recover motion between them. Reference errors below are evaluated within each component.</div>
      </section>

      <section class="native-timings" v-if="benchmark">
        <div class="timing-heading"><div><span class="section-label">MEASURED C++ PERFORMANCE</span><h2>GTSAM solves it. The browser replays it.</h2></div><span class="native-badge">{{environment.buildType}} · {{benchmark.repetitions}} runs · {{benchmark.warmup}} warmups</span></div>
        <div class="timing-phases"><div v-for="phase in phaseTimings" :key="phase.id" :class="{featured:phase.id==='optimization'}"><span>{{phase.title}}</span><strong>{{phase.stats.medianMs.toFixed(2)}} <small>ms</small></strong><p>{{phase.detail}}</p><span class="timing-range">min {{phase.stats.minMs.toFixed(2)}} · p95 {{phase.stats.p95Ms.toFixed(2)}} ms</span></div></div>
        <div class="timing-context"><p>{{recording.hardware.cpu}} <i>·</i> GTSAM {{environment.gtsamVersion}} <i>·</i> {{environment.linearSolver}}<br/>Native CPU timings; FK extraction, dense trajectory expansion, JSON, network, history capture, and rendering are excluded.</p><a :href="`${baseUrl}${sequence.recording}`" download>Download native results & timings ↓</a></div>
        <div class="timing-comparison"><span v-for="entry in modelTimings" :key="entry.id" :class="{active:entry.id===model}">{{entry.title}} <b>{{entry.run.benchmark.medianMs.toFixed(2)}} ms</b><small>{{entry.run.result.iterations}} LM iterations</small></span></div>
      </section>

      <section class="results-grid">
        <article class="solve-card"><span class="section-label">RECORDED NATIVE SOLVE</span><div class="solve-header"><h2>Fit the shared contacts.</h2><button class="solve-button" @click="replayNative">{{replaying?'Pause native replay':'Replay GTSAM solve'}} <span>↻</span></button></div>
          <div class="solve-metrics"><div><strong>{{(currentRms*1000).toFixed(2)}}<small> mm</small></strong><span>contact residual RMS</span></div><div><strong>{{Number(benchmark?.medianMs||0).toFixed(2)}}<small> ms</small></strong><span>native GTSAM · median</span></div></div>
          <div class="loss-row"><svg viewBox="0 0 286 70" aria-label="Log objective versus native GTSAM iteration"><path :d="lossPath" fill="none" stroke="#267a65" stroke-width="2"/></svg><span>{{result.iterations}}<small>LM iterations</small></span></div>
          <label class="iteration-control">Iteration <input type="range" aria-label="Solver iteration" min="0" :max="Math.max(0,history.length-1)" v-model.number="iteration" @input="replaying=false"/><span class="mono">{{iterationNumber}}</span></label>
          <p>C++ GTSAM jointly optimizes the event poses and {{graph.landmarks.length}} contact landmarks using Levenberg–Marquardt and sparse Cholesky{{model==='ground'?'; Point2 landmarks enforce the level floor':''}}. {{observationCount}} contact factors carry the geometry. <b>{{result.converged?'Converged under the recorded tolerance.':'Iteration budget reached; convergence is not claimed.'}}</b></p><button class="export-button" @click="exportInitialization">Export initialization JSON <span>↓</span></button>
        </article>
        <article class="evaluation-card"><span class="section-label">HELD-OUT REFERENCE</span><h2>{{componentCount>1?'Accuracy within support components':'How close is the motion?'}}</h2><div class="solve-metrics"><div><strong>{{comparison.position.toFixed(1)}}<small> mm</small></strong><span>{{componentCount>1?'component-relative':'root position'}} RMSE</span></div><div><strong>{{comparison.angle.toFixed(2)}}<small> °</small></strong><span>root orientation RMSE</span></div></div><p>Recorded floating-base poses are used only for this comparison and the ghost overlay. They never enter the native solver.</p><div class="evaluation-footnote">Reference aligned at {{componentCount>1?'each component’s first event':'the first event'}}. Errors include slip, sole rocking, and contact-model mismatch. {{componentCount>1?'No global trajectory accuracy is claimed across unconnected supports.':''}}</div></article>
        <article class="insight-card" :class="{warning:model==='point'||componentCount>1}"><span class="section-label">WHAT MAKES IT WORK</span><h2>{{model==='point'?'A point is not a full pose.':'The contact must constrain a frame.'}}</h2><div class="rank-diagram"><span :class="{chosen:model==='point'}"><b>1 point</b><strong>3 / 6</strong><small>pose rank</small></span><span><b>2 points</b><strong>5 / 6</strong><small>pose rank</small></span><span :class="{chosen:model!=='point'}"><b>3 noncollinear</b><strong>6 / 6</strong><small>pose rank</small></span></div><p>Ranks assume known world landmarks. Connected flat-foot patches transmit a full pose; one point per foot leaves rotational ambiguity even after fixing X₀.</p><a href="#method">Read the assumptions and method <span>↓</span></a></article>
      </section>

      <section id="method" class="method"><div><span class="section-label">AN ALTERNATIVE INITIALIZER</span><h2>Measured joints in.<br/>GTDynamics Values out.</h2></div><div class="method-copy"><p>The reusable C++ ContactInitializer builds a GTSAM factor graph from frozen FK at contact boundaries. Its GTDynamics adapter, Initializer::InitializeSolutionContacts, keeps the familiar robot, root-link name, initial root pose, and Values return type. It adds measured joint samples and an explicit stance schedule with a fresh ID for every touchdown.</p><p>GTSAM optimizes Pose3 event poses and Point3 contact landmarks. The level-ground variant uses Point2 landmark coordinates with fixed height. After the event solve, the adapter interpolates root seeds to requested samples and runs FK to return standard PoseKey link poses, preserving measured joints. Dense expansion is an initialization approximation, not an added observation.</p><p>This is an alternative to inverse-kinematics initialization when joint measurements are available. An estimator can consume the solved root pose as its existing pose seed while retaining its own twist and bias initialization. Contact phases remain an input; disconnected support chains need additional information, and point contacts leave attitudes ambiguous.</p></div><div class="method-facts"><span>{{data.model.jointNames?.length||29}} fixed joint measurements</span><span>{{data.metadata.sampleRateHz}} Hz source recording</span><span>{{model==='point'?1:3}} sole point{{model==='point'?'':'s'}} per stance</span><span>{{componentCount}} anchored component{{componentCount>1?'s':''}}</span><a :href="sequence.sourceUrl" target="_blank" rel="noopener">Recording & model provenance ↗</a><a href="THIRD_PARTY_LICENSES.txt" target="_blank">Third-party license notices ↗</a></div></section>
      <footer><span>GTDYNAMICS / CONTACT INITIALIZATION</span><span>Small graph. Explicit assumptions. A starting point for the full dynamics.</span></footer>
    </template>
  </main>
</template>
