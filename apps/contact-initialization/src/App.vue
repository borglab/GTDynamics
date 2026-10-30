<script setup>
import { computed, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue';
import { ContactScene } from './scene.js';
import { buildGraph, optimizeGraph, quaternionToRotation, composePoses, inversePose, transformPoint } from './graph.js';

const data = ref(null), graph = ref(null), result = ref(null), error = ref('');
const sceneHost = ref(null), graphHost = ref(null);
const stage = ref(2), model = ref('ground'), selected = ref(24), iteration = ref(0);
const showReference = ref(true), showGhosts = ref(true), playing = ref(false), solving = ref(false);
let scene, clock, lastTick = 0;
const steps = [
  {title:'Find the events', detail:'A stance begins. A stance ends. Keep those instants.'},
  {title:'Freeze the robot', detail:'Joint angles become rigid geometry, computed once with FK.'},
  {title:'Connect the contacts', detail:'Shared stance landmarks stitch the rigid frames into a walk.'},
];
const frames = computed(() => graph.value?.frames || []);
const contacts = computed(() => graph.value?.contacts || []);
const current = computed(() => frames.value[selected.value]);
const sample = computed(() => data.value?.samples[current.value?.sampleIndex]);
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
  const alignment = composePoses(graph.value.initialPoses[0], inversePose(references[0]));
  return references.map(p => composePoses(alignment, p));
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
const ticks = computed(() => Array.from({length:9},(_,i)=>i));
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
    floorConstrained:model.value==='ground'||stage.value===0,mode:stage.value===1?'frozen':'solved'};
});
function updateScene() {
  if (!scene || !data.value || !graph.value || !result.value) return;
  scene.update({...sceneGeometry.value, selectedIndex:selected.value,
    showReference:showReference.value&&stage.value===2,showGhosts:showGhosts.value&&stage.value!==1});
}
async function solve(perturb=false) {
  if (!data.value) return;
  solving.value=true; playing.value=false;
  await new Promise(resolve=>setTimeout(resolve,20));
  try {
    const g=buildGraph(data.value,{model:model.value==='point'?'point':'patch',groundPlane:model.value==='ground'});
    if(perturb) g.initialPoses=g.initialPoses.map((p,i)=> i===0 ? p : ({rotation:p.rotation.slice(),translation:p.translation.map((v,j)=>v+0.035*Math.sin(i*2.3+j*1.7))}));
    graph.value=g;
    result.value=optimizeGraph(g,{iterations:100});
    iteration.value=Math.max(0,result.value.history.length-1);
    selected.value=Math.min(selected.value,g.frames.length-1);
    await nextTick();selectFrame(selected.value);updateScene();scene?.reset();
  } catch(e) { error.value=e.message; }
  finally {solving.value=false;}
}
function selectFrame(index) {
  selected.value=Math.max(0,Math.min(frames.value.length-1,Number(index)));
  const el=graphHost.value;
  if(el) el.scrollTo({left:Math.max(0,frameX(selected.value)-el.clientWidth/2),behavior:'smooth'});
}
function exportInitialization() {
  const payload={schemaVersion:1,source:data.value.metadata.sourcePr,sourceRecordingSha256:data.value.metadata.recordingSha256,
    model:model.value,assumptions:{stationaryMaterialPoints:true,groundPlane:graph.value.groundPlane||null,
      gauge:'First event pose fixed; reference root trajectory excluded.'},
    observability:graph.value.observability,contactRmsMeters:result.value.residualRms,
    eventPoses:frames.value.map((f,i)=>({sampleIndex:f.sampleIndex,time:f.time,...result.value.poses[i]})),
    contacts:contacts.value.map(c=>({id:c.id,foot:c.foot,start:c.start,end:c.end,points:c.landmarkIndices.map(i=>result.value.landmarks[i])}))};
  const url=URL.createObjectURL(new Blob([JSON.stringify(payload,null,2)],{type:'application/json'}));
  const a=document.createElement('a');a.href=url;a.download=`g1-contact-initialization-${model.value}.json`;a.click();
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
  clock=requestAnimationFrame(animate);
}
watch([selected,iteration,stage,showReference,showGhosts],()=>nextTick(updateScene));
watch(model,()=>solve());
onMounted(async()=>{
  try {
    const response=await fetch(`${import.meta.env.BASE_URL}g1-walk.json`);
    if(!response.ok)throw new Error(`G1 recording could not load (${response.status}).`);
    data.value=await response.json();
    await solve();await nextTick();
    scene=new ContactScene(sceneHost.value,{onSelect:selectFrame});updateScene();
    clock=requestAnimationFrame(animate);
  }catch(e){error.value=e.message;}
});
onBeforeUnmount(()=>{cancelAnimationFrame(clock);scene?.dispose();});
</script>

<template>
  <main class="lab" :data-model="model" :data-event="selected" :data-iteration="iteration">
    <header class="intro">
      <div><p class="eyebrow"><span class="live-dot"></span> A WALKING INITIALIZER / UNITREE G1</p>
        <h1>Walking, reduced to <em>contact.</em></h1>
        <p class="intro-copy">Freeze the joint geometry. Keep the contact events. Let a small factor graph connect the walk.</p>
      </div>
      <div class="source-stamp"><span class="stamp-icon">G1</span><div><strong>One real walk. Fewer unknowns.</strong><span>MJLab policy rollout · 100 Hz · 8 seconds</span><a href="https://github.com/borglab/GTDynamics/pull/446" target="_blank" rel="noopener">Sequence from PR #446 ↗</a></div></div>
    </header>

    <div v-if="error" class="error-banner" role="alert">{{error}}</div>
    <div v-if="!result && !error" class="loading">Loading the G1 sequence and assembling contact geometry…</div>
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
            <g v-for="(frame,i) in frames" :key="frame.id" class="pose-node" :class="{selected:i===selected,anchor:i===0}" role="button" tabindex="0" :aria-label="`Select event ${i}, ${time(frame.time)} seconds`" @click="selectFrame(i)" @keydown.enter="selectFrame(i)" @keydown.space.prevent="selectFrame(i)"><circle :cx="frameX(i)" cy="49" r="14"/><text :x="frameX(i)" y="53">X<tspan font-size="8" dy="3">{{i}}</tspan></text><text :x="frameX(i)" y="83" class="node-time">{{time(frame.time)}}</text><path v-if="i===0" :d="`M${frameX(i)-6} 29h12m-6 0v-7`" class="anchor-mark"/></g>
            <g v-for="contact in contacts" :key="contact.id" class="contact-node" :class="[contact.foot,{active:activeIds.includes(contact.id)}]"><rect :x="contactX(contact)-13" :y="contactY(contact)-10" width="26" height="20" rx="5"/><text :x="contactX(contact)" :y="contactY(contact)+4">{{contactLabel(contact)}}</text></g>
          </svg></div>
          <div class="graph-key"><span><i class="circle-key"></i>Pose variable</span><span><i class="square-key"></i>Contact factor</span><span class="anchor-key">X₀ fixed</span></div>
          <div class="equation"><span class="equation-main">Rᵢ pᵢⱼ + tᵢ − cⱼ = 0</span><p><b>pᵢⱼ</b> is known from forward kinematics.<br/><b>Xᵢ = (Rᵢ, tᵢ)</b> and <b>cⱼ</b> are the unknowns.</p></div>
          <div class="model-picker"><label for="contact-model">CONTACT GEOMETRY</label><select id="contact-model" v-model="model" aria-label="Contact geometry"><option value="ground">Sole patch + level ground</option><option value="patch">Sole patch · contacts only</option><option value="point">One foot point · ambiguous</option></select></div>
          <p class="model-note">{{model==='ground'?'Adds a level-floor assumption: all stance landmarks lie on one horizontal plane. Sole geometry sets the initial tilt; this extra constraint limits rocking-induced drift.':model==='patch'?'Three noncollinear material points constrain a rigid foot patch. G1 rocks its feet, so this pure contact model accumulates large attitude drift.':'One fixed point leaves three rotations free; two leave rotation about their connecting line. A low residual cannot certify the full pose.'}}</p>
        </aside>
      </section>

      <section class="timeline-card">
        <div class="timeline-heading"><div><span class="section-label">THE CONTACT CLOCK</span><h2>A frame at every touchdown and lift-off.</h2></div><span class="timeline-meta">{{data.samples.length}} encoder samples → {{frames.length}} event frames <i>·</i> {{(data.samples.length/frames.length).toFixed(1)}}× fewer poses</span></div>
        <svg class="timeline" viewBox="0 0 1200 145" @click="clickTimeline" role="img" aria-label="Left and right stance intervals over eight seconds; click to select the nearest event">
          <g v-for="tick in ticks" :key="tick"><line :x1="timelineX(tick)" y1="18" :x2="timelineX(tick)" y2="122" class="timeline-grid"/><text :x="timelineX(tick)" y="140" text-anchor="middle" class="svg-label">{{tick}}s</text></g>
          <text x="8" y="48" class="timeline-side left-fill">LEFT</text><text x="8" y="99" class="timeline-side right-fill">RIGHT</text>
          <rect x="68" y="26" width="1100" height="36" rx="4" class="timeline-lane"/><rect x="68" y="77" width="1100" height="36" rx="4" class="timeline-lane"/>
          <g v-for="contact in contacts" :key="contact.id" :class="['stance',contact.foot,{active:activeIds.includes(contact.id)}]"><rect :x="timelineX(data.samples[contact.start]?.t)" :y="contact.foot==='left'?26:77" :width="Math.max(2,timelineX(data.samples[contact.end]?.t)-timelineX(data.samples[contact.start]?.t))" height="36" rx="3"/><text v-if="contact.end-contact.start>15" :x="timelineX((data.samples[contact.start]?.t+data.samples[contact.end]?.t)/2)" :y="contact.foot==='left'?47:98" text-anchor="middle">{{contactLabel(contact)}}</text></g>
          <path :d="loadPath('left')" class="load-trace left"/><path :d="loadPath('right')" class="load-trace right"/>
          <line v-for="frame in frames" :key="frame.id" :x1="timelineX(frame.time)" y1="18" :x2="timelineX(frame.time)" y2="120" class="event-tick"/>
          <line :x1="timelineX(current?.time)" y1="10" :x2="timelineX(current?.time)" y2="122" class="playhead"/><circle :cx="timelineX(current?.time)" cy="10" r="4" class="playhead-dot"/>
        </svg>
        <div class="timeline-caption"><span>Filled bars: stance episodes <i>·</i> Trace: replayed normal load <i>·</i> Vertical ticks: frozen frames</span><span>Contact schedule is an input, derived here from simulator forces.</span></div>
      </section>

      <section class="results-grid">
        <article class="solve-card"><span class="section-label">A SMALL, REAL OPTIMIZATION</span><div class="solve-header"><h2>Fit the shared contacts.</h2><button class="solve-button" :disabled="solving" @click="solve(true)">{{solving?'Solving…':'Perturb & solve'}} <span>↻</span></button></div>
          <div class="solve-metrics"><div><strong>{{(currentRms*1000).toFixed(2)}}<small> mm</small></strong><span>contact residual RMS</span></div><div><strong>{{Number(result.timeMs).toFixed(1)}}<small> ms</small></strong><span>this browser · solve only</span></div></div>
          <div class="loss-row"><svg viewBox="0 0 286 70" aria-label="Log objective versus solver iteration"><path :d="lossPath" fill="none" stroke="#267a65" stroke-width="2"/></svg><span>{{history.length-1}}<small>iterations</small></span></div>
          <label class="iteration-control">Iteration <input type="range" aria-label="Solver iteration" min="0" :max="Math.max(0,history.length-1)" v-model.number="iteration"/><span class="mono">{{iteration}}</span></label>
          <p>Block updates fit rigid poses and average shared landmarks{{model==='ground'?', projected onto the level floor':''}}. The first pose anchors the world; {{observationCount}} 3D contact factors carry the geometry.</p><button class="export-button" @click="exportInitialization">Export initialization JSON <span>↓</span></button>
        </article>
        <article class="evaluation-card"><span class="section-label">HELD-OUT REFERENCE</span><h2>How close is the walk?</h2><div class="solve-metrics"><div><strong>{{comparison.position.toFixed(1)}}<small> mm</small></strong><span>root position RMSE</span></div><div><strong>{{comparison.angle.toFixed(2)}}<small> °</small></strong><span>root orientation RMSE</span></div></div><p>Recorded floating-base poses are used only for this comparison and the ghost overlay. They never enter the graph or its initialization.</p><div class="evaluation-footnote">Reference aligned at the first event. Errors include slip, sole rocking, and contact-model mismatch. {{componentCount>1?`${componentCount} disconnected components have independent gauges; a global walk is not observable.`:''}}</div></article>
        <article class="insight-card" :class="{warning:model==='point'||componentCount>1}"><span class="section-label">WHAT MAKES IT WORK</span><h2>{{model==='point'?'A point is not a full pose.':'The contact must constrain a frame.'}}</h2><div class="rank-diagram"><span :class="{chosen:model==='point'}"><b>1 point</b><strong>3 / 6</strong><small>pose rank</small></span><span><b>2 points</b><strong>5 / 6</strong><small>pose rank</small></span><span :class="{chosen:model!=='point'}"><b>3 noncollinear</b><strong>6 / 6</strong><small>pose rank</small></span></div><p>Ranks assume known world landmarks. Connected flat-foot patches transmit a full pose; one point per foot leaves rotational ambiguity even after fixing X₀.</p><a href="#method">Read the assumptions and method <span>↓</span></a></article>
      </section>

      <section id="method" class="method"><div><span class="section-label">THE IDEA, MADE PRECISE</span><h2>Keep the geometry.<br/>Shrink the state.</h2></div><div class="method-copy"><p>At each contact boundary, forward kinematics turns the measured joint angles into a rigid constellation of body and sole points in the pelvis frame. Joint angles and individual link poses are constants. The only optimized quantities are the event poses and the world locations of the stance landmarks.</p><p>A fresh landmark group belongs to each stance, including separate touchdowns by the same foot. Every event that falls inside that stance observes the same group. The pure model contains contact factors and an initial gauge anchor. The default adds an explicit level-ground constraint, cⱼ,z = h. There is no motion prior or reference trajectory factor.</p><p>Contact cannot generally be inferred from joint angles alone. This example uses 20 N / 10 N force hysteresis, bridges 15 isolated 10 ms dropouts, and excludes the initial unsupported sample. Endpoints use the first and last supported samples. G1 rocks about 11° per stance on median: rigid patches accumulate attitude drift, while level ground helps without removing horizontal drift. Flight or missing support overlap breaks the chain.</p></div><div class="method-facts"><span>29 measured joints</span><span>100 Hz source recording</span><span>3 sole points per stance</span><span>1 anchored initial pose</span><a :href="`${data.metadata?.recorder}`" target="_blank" rel="noopener">Recording & model provenance ↗</a><a href="THIRD_PARTY_LICENSES.txt" target="_blank">Third-party license notices ↗</a></div></section>
      <footer><span>GTDYNAMICS / CONTACT INITIALIZATION</span><span>Small graph. Explicit assumptions. A starting point for the full dynamics.</span></footer>
    </template>
  </main>
</template>
