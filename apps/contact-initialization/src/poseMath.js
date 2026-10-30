// Coordinate transforms for displaying and evaluating recorded native results.
// Optimization and graph initialization are implemented in C++.
const dot = (a,b) => a.reduce((sum,v,i)=>sum+v*b[i],0);
const transpose = r => [r[0],r[3],r[6],r[1],r[4],r[7],r[2],r[5],r[8]];
const rotate = (r,p) => [dot(r.slice(0,3),p),dot(r.slice(3,6),p),dot(r.slice(6,9),p)];
export function quaternionToRotation(q) {
  const length=Math.hypot(...q);
  if(!(length>0))throw new Error('A recorded quaternion must be nonzero.');
  const [w,x,y,z]=q.map(v=>v/length);
  return [1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)];
}
export const transformPoint = (pose,p) => rotate(pose.rotation,p).map((v,i)=>v+pose.translation[i]);
export function composePoses(a,b) {
  const bt=transpose(b.rotation);
  return {rotation:Array.from({length:9},(_,i)=>dot(a.rotation.slice(3*Math.floor(i/3),3*Math.floor(i/3)+3),bt.slice(3*(i%3),3*(i%3)+3))),translation:transformPoint(a,b.translation)};
}
export function inversePose(pose) {
  const rotation=transpose(pose.rotation);
  return {rotation,translation:rotate(rotation,pose.translation).map(v=>-v)};
}
