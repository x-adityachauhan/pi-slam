"use strict";
const $ = s => document.querySelector(s);
const canvas = $("#map");
const gl = canvas.getContext("webgl", {antialias:true, alpha:false});
let state = {points:[], trajectory:[]}, buffers = null, center = [0,0,0], extent = 1;
let yaw = 0, pitch = 0, zoom = 1, drag = null, cameraMode = true, requestSerial = 0;
const I = () => [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
const mul = (a,b) => a.map(row => b[0].map((_,j) => row.reduce((v,x,k) => v+x*b[k][j],0)));
let program, locations;
function error(message) { $("#error").textContent=message; $("#error").style.display="block"; }
function shader(type, source) {
  const s=gl.createShader(type);gl.shaderSource(s,source);gl.compileShader(s);
  if(!gl.getShaderParameter(s,gl.COMPILE_STATUS)) throw Error(gl.getShaderInfoLog(s));
  return s;
}
function initialize() {
  if(!gl) throw Error("WebGL is unavailable. Enable browser graphics acceleration to view the 3D map.");
  program=gl.createProgram();
  gl.attachShader(program,shader(gl.VERTEX_SHADER,
    "attribute vec3 p; attribute vec3 rgb; uniform mat4 mvp; uniform float size; varying vec3 color; void main(){ gl_Position=mvp*vec4(p,1.0); gl_PointSize=size; color=rgb; }"));
  gl.attachShader(program,shader(gl.FRAGMENT_SHADER,
    "precision mediump float; varying vec3 color; uniform bool dots; void main(){ if(dots && distance(gl_PointCoord,vec2(0.5))>0.5) discard; gl_FragColor=vec4(color,1.0); }"));
  gl.linkProgram(program);
  if(!gl.getProgramParameter(program,gl.LINK_STATUS)) throw Error(gl.getProgramInfoLog(program));
  gl.useProgram(program);
  locations={p:gl.getAttribLocation(program,"p"),rgb:gl.getAttribLocation(program,"rgb"),
    mvp:gl.getUniformLocation(program,"mvp"),size:gl.getUniformLocation(program,"size"),dots:gl.getUniformLocation(program,"dots")};
  gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LEQUAL);
}
function gpuBuffer(data) {
  const b=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,b);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(data),gl.STATIC_DRAW);return b;
}
function upload() {
  if(!gl) return;
  if(buffers) Object.values(buffers).forEach(b=>gl.deleteBuffer(b));
  const colors=state.colors?.length===state.points.length ? state.colors.flat().map(x=>x/255) : state.points.flatMap(()=>[.4,.84,.8]);
  buffers={points:gpuBuffer(state.points.flat()),colors:gpuBuffer(colors),
    path:gpuBuffer(state.trajectory.flat()),pathColors:gpuBuffer(state.trajectory.flatMap(()=>[1,.77,.45])),
    mesh:gpuBuffer((state.faces||[]).flatMap(f=>f.flatMap(i=>state.points[i]))),
    meshColors:gpuBuffer((state.faces||[]).flatMap(f=>f.flatMap(i=>state.colors[i].map(c=>c/255))))};
}
function fit() {
  const p=state.points.length?state.points:state.trajectory;if(!p.length)return;
  const axes=[0,1,2].map(k=>p.map(v=>v[k]).sort((a,b)=>a-b));
  const lo=axes.map(v=>v[Math.floor(v.length*.02)]),hi=axes.map(v=>v[Math.min(v.length-1,Math.floor(v.length*.98))]);
  center=lo.map((v,k)=>(v+hi[k])/2);extent=Math.max(...hi.map((v,k)=>v-lo[k]),.01);
  zoom=1;yaw=0;pitch=0;
}
function viewMatrix() {
  const T=state.reference?.T_world_camera;
  if(cameraMode && T) {
    const v=I();
    for(let r=0;r<3;r++){for(let c=0;c<3;c++)v[r][c]=T[c][r];v[r][3]=-v[r].slice(0,3).reduce((s,x,k)=>s+x*T[k][3],0);}
    return v;
  }
  const base=I();
  if(T)for(let r=0;r<3;r++)for(let c=0;c<3;c++)base[r][c]=T[c][r];
  const cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);
  const R=mul([[1,0,0,0],[0,cp,-sp,0],[0,sp,cp,0],[0,0,0,1]],mul([[cy,0,-sy,0],[0,1,0,0],[sy,0,cy,0],[0,0,0,1]],base));
  for(let r=0;r<3;r++)R[r][3]=-R[r].slice(0,3).reduce((s,x,k)=>s+x*center[k],0);
  R[2][3]+=extent*1.65/zoom;return R;
}
function projection() {
  const near=extent*.0005,far=extent*100;
  let sx,sy,ox=0,oy=0;
  if(cameraMode && state.camera) {
    const c=state.camera,s=Math.min(canvas.width/c.width,canvas.height/c.height)*zoom;
    sx=2*c.K[0][0]*s/canvas.width;sy=2*c.K[1][1]*s/canvas.height;
    ox=2*(c.K[0][2]-c.width/2)*s/canvas.width;oy=-2*(c.K[1][2]-c.height/2)*s/canvas.height;
  }else{sy=2.1;sx=sy*canvas.height/canvas.width;}
  return [[sx,0,ox,0],[0,-sy,oy,0],[0,0,(far+near)/(far-near),-2*near*far/(far-near)],[0,0,1,0]];
}
function bind(p,c) {
  gl.bindBuffer(gl.ARRAY_BUFFER,p);gl.vertexAttribPointer(locations.p,3,gl.FLOAT,false,0,0);gl.enableVertexAttribArray(locations.p);
  gl.bindBuffer(gl.ARRAY_BUFFER,c);gl.vertexAttribPointer(locations.rgb,3,gl.FLOAT,false,0,0);gl.enableVertexAttribArray(locations.rgb);
}
function draw() {
  if(!gl || !program || !buffers)return;
  const ratio=Math.min(devicePixelRatio||1,2);
  const w=Math.round(canvas.clientWidth*ratio),h=Math.round(canvas.clientHeight*ratio);
  if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h;}
  gl.viewport(0,0,w,h);gl.clearColor(.06,.09,.13,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
  const m=mul(projection(),viewMatrix());
  gl.uniformMatrix4fv(locations.mvp,false,new Float32Array(m[0].flatMap((_,c)=>m.map(row=>row[c]))));
  gl.uniform1f(locations.size,Number($("#point-size").value)*ratio);gl.uniform1i(locations.dots,1);
  bind(buffers.points,buffers.colors);gl.drawArrays(gl.POINTS,0,state.points.length);
  if($("#surfaces").checked && state.faces?.length) {
    bind(buffers.mesh,buffers.meshColors);gl.uniform1i(locations.dots,0);gl.drawArrays(gl.TRIANGLES,0,state.faces.length*3);
  }
  if($("#path").checked && state.trajectory.length) {
    bind(buffers.path,buffers.pathColors);gl.uniform1i(locations.dots,0);gl.drawArrays(gl.LINE_STRIP,0,state.trajectory.length);
  }
  $("#camera-view").classList.toggle("active",cameraMode);
  $("#reset").classList.toggle("active",!cameraMode);
  const top=$("#topdown"),ctx=top.getContext("2d");
  top.width=Math.round(top.clientWidth);top.height=300;
  ctx.fillStyle="#101722";ctx.fillRect(0,0,top.width,top.height);
  const scale=Math.min(top.width,top.height)*.8/extent;
  const xy=p=>[top.width/2+(p[0]-center[0])*scale,top.height/2+(p[2]-center[2])*scale];
  ctx.fillStyle="#70d9e8";
  for(const p of state.points){const [x,y]=xy(p);ctx.fillRect(x,y,2,2);}
  ctx.strokeStyle="#ffc477";ctx.beginPath();
  state.trajectory.forEach((p,i)=>{const [x,y]=xy(p);i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();
}
async function refresh(mapId) {
  const serial=++requestSerial;
  try {
    const query=mapId?"?map="+encodeURIComponent(mapId):"";
    const response=await fetch("/api"+query,{cache:"no-store"});
    if(!response.ok)throw Error("Map is unavailable");
    const next=await response.json();if(serial!==requestSerial)return;
    const first=!state.points.length||state.map_id!==next.map_id;
    state=next;upload();if(first){fit();cameraMode=!!state.reference;}
    $("#error").style.display="none";
    $("#status").textContent=state.status;
    $("#title").textContent=state.saved?"Pi camera · saved 3D reconstruction":"Pi camera · live SLAM";
    $("#camera-label").textContent=state.saved?(state.reference_label||"Recorded camera view"):"Live camera";
    $("#map-label").textContent=state.sparse?"Sparse 3D landmarks":state.saved?"Reconstructed surfaces":"Live 3D landmarks";
    $("#surfaces").disabled=Boolean(state.sparse);
    $("#stats").textContent=state.saved?state.total_points.toLocaleString()+(state.sparse?" landmarks · ":" surface points · ")+(state.faces?.length||0).toLocaleString()+" triangles · "+(state.quality||"") :
      (state.frames||0)+" frames · "+Number(state.fps||0).toFixed(1)+" FPS · "+Number(state.tracked_pct||0).toFixed(1)+"% tracked · "+(state.keypoints||0)+" keypoints · "+(state.map_count||0)+" maps · "+(state.skipped||0)+" skipped";
    const select=$("#maps");select.hidden=!(state.maps?.length);
    select.replaceChildren(...(state.maps||[]).map(m=>{const o=document.createElement("option");o.value=m.id;o.textContent=m.id+" · "+m.points.toLocaleString()+" points";o.selected=m.id===state.map_id;return o;}));
    $("#camera").src="/frame.jpg"+query+(query?"&":"?")+"t="+Date.now();
    $("#download").hidden=!state.saved||!state.maps;
    $("#download").href="/download?map="+encodeURIComponent(state.map_id||"");
    $("#download").download=(state.map_id||"map")+".ply";
    $("#notice").textContent=state.saved?"Each map has its own coordinate frame. Missing surfaces remain empty; scale is relative. "+(state.method||""):"";
    draw();
    // Saved maps are immutable: do not repeatedly transfer and parse a large cloud.
    if(!state.saved)setTimeout(()=>refresh(),250);
  }catch(e){error(e.message);$("#status").textContent="Session stopped or map unavailable";setTimeout(()=>refresh(),1000);}
}
canvas.onpointerdown=e=>{if(cameraMode){cameraMode=false;zoom=1;}drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);draw();};
canvas.onpointermove=e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*.008;pitch=Math.max(-1.5,Math.min(1.5,pitch+(e.clientY-drag[1])*.008));drag=[e.clientX,e.clientY];draw();};
canvas.onpointerup=canvas.onpointercancel=()=>{drag=null;};
canvas.onwheel=e=>{e.preventDefault();zoom=Math.max(.1,Math.min(20,zoom*Math.exp(-e.deltaY*.001)));draw();};
$("#reset").onclick=()=>{cameraMode=false;fit();draw();};
$("#camera-view").onclick=()=>{cameraMode=!!state.reference;zoom=1;draw();};
$("#maps").onchange=e=>refresh(e.target.value);
$("#point-size").oninput=$("#path").onchange=$("#surfaces").onchange=draw;
window.onresize=draw;
try{initialize();refresh();}catch(e){error(e.message);}
