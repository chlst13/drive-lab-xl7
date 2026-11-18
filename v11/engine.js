/* Drive Lab V11 — browser-only 2D driving simulator.
   Steering: ±720 degrees wheel mapped to ±32 degrees tyre.
   Bicycle approximation for low-speed parking only, not a validated real vehicle model. */
(function(){
"use strict";
const $=id=>document.getElementById(id),pack=window.DriveLevels;
if(!pack){$('engineStatus').textContent="DATA LEVEL GAGAL DIMUAT";return;}
const {CAR,scenes}=pack,canvas=$('sceneCanvas'),ctx=canvas.getContext("2d",{alpha:false});
if(!ctx){$('engineStatus').textContent="CANVAS TIDAK TERSEDIA";return;}
const PI=Math.PI,TWO=PI*2,RAD=PI/180;
const clamp=(x,a,b)=>Math.max(a,Math.min(x,b));
const rad=d=>d*RAD,deg=d=>d/RAD,wrap=a=>Math.atan2(Math.sin(a),Math.cos(a));
const fx=n=>n.toFixed(1).replace(".",",");
const fp=(p,u,v)=>({x:p.x+Math.cos(p.a)*u-Math.sin(p.a)*v,y:p.y+Math.sin(p.a)*u+Math.cos(p.a)*v});
const vehicleHull=p=>[fp(p,-CAR.rear,-CAR.half),fp(p,CAR.front,-CAR.half),fp(p,CAR.front,CAR.half),fp(p,-CAR.rear,CAR.half)];
function mirrorHull(p,side){const c=side*CAR.mirrors,d=.22,u=2.63;return [fp(p,u-d,c-.1),fp(p,u+d,c-.1),fp(p,u+d,c+.1),fp(p,u-d,c+.1)];}
const rectHull=o=>[{x:o.x,y:o.y},{x:o.x+o.w,y:o.y},{x:o.x+o.w,y:o.y+o.h},{x:o.x,y:o.y+o.h}];
function obstacleHull(o){
 if(o.type==="block")return rectHull(o);
 if(o.type==="bike"){const p={x:o.cx,y:o.cy,a:o.a};return [fp(p,-.95,-.35),fp(p,.95,-.35),fp(p,.95,.35),fp(p,-.95,.35)];}
 const rear={x:o.cx-Math.cos(o.a)*1.385,y:o.cy-Math.sin(o.a)*1.385,a:o.a};
 return vehicleHull(rear);
}
function projection(poly,nx,ny){let lo=Infinity,hi=-Infinity;for(const p of poly){const a=nx*p.x+ny*p.y;lo=Math.min(lo,a);hi=Math.max(hi,a);}return{lo,hi};}
function overlap(a,b){
 for(const poly of[a,b])for(let i=0;i<poly.length;i++){
 const p=poly[i],q=poly[(i+1)%poly.length],nx=q.y-p.y,ny=p.x-q.x,ap=projection(a,nx,ny),bp=projection(b,nx,ny);
 if(ap.hi<=bp.lo+.00001||bp.hi<=ap.lo+.00001)return false;
 }
 return true;
}
const state={level:scenes[0],pose:{...scenes[0].start},wheel:0,tyre:0,speed:0,intent:0,pedalPointers:new Map(),keys:new Set(),wheelPointer:null,wheelLastAngle:0,mode:"precise",camera:"map",assist:true,lines:true,corners:true,paused:false,hits:0,gearChanges:0,lastDrive:0,elapsed:0,lastImpact:-10,lastTime:0,lastHud:0,won:false,trail:[],view:{ox:0,oy:0,s:1},canvasW:700,canvasH:400,dpr:1,edgeMemory:[],fatal:false};
function onPavement(x,y){return state.level.pavement.some(r=>x>=r.x-0.00001&&y>=r.y-0.00001&&x<=r.x+r.w+0.00001&&y<=r.y+r.h+0.00001);}
function shapeOnPavement(poly){
 for(let i=0;i<poly.length;i++){
 const a=poly[i],b=poly[(i+1)%poly.length],n=Math.max(4,Math.ceil(Math.hypot(a.x-b.x,a.y-b.y)/.105));
 for(let j=0;j<=n;j++){const u=j/n;if(!onPavement(a.x*(1-u)+b.x*u,a.y*(1-u)+b.y*u))return false;}
 }
 return true;
}
function isBlocked(p){
 const shapes=[vehicleHull(p),mirrorHull(p,-1),mirrorHull(p,1)];
 for(const sh of shapes)if(!shapeOnPavement(sh))return true;
 for(const o of state.level.obstacles){const op=obstacleHull(o);for(const sh of shapes)if(overlap(sh,op))return true;}
 return false;
}
function nextPose(p,v,tyre,dt){
 const yaw=v/CAR.wheelbase*Math.tan(rad(tyre))*dt,a=p.a+yaw/2;
 return{x:p.x+v*Math.cos(a)*dt,y:p.y+v*Math.sin(a)*dt,a:wrap(p.a+yaw)};
}
function sensorRay(p,ang,reach=5.5){
 const ox=p.x,oy=p.y,dx=Math.cos(ang),dy=Math.sin(ang);
 for(let d=0;d<reach;d+=.11){
 const x=ox+dx*d,y=oy+dy*d;if(!onPavement(x,y))return d;
 const dot={x,y};
 for(const o of state.level.obstacles)if(pointInPoly(dot,obstacleHull(o)))return d;
 }
 return reach;
}
function pointInPoly(pt,poly){let v=false;for(let i=0,j=poly.length-1;i<poly.length;j=i++){
 const a=poly[i],b=poly[j];
 if((a.y>pt.y)!==(b.y>pt.y)&&pt.x<(b.x-a.x)*(pt.y-a.y)/(b.y-a.y)+a.x)v=!v;
 }return v;
}
function measure(){
 const p=state.pose;
 const arr=[
 ["sensorF",fp(p,CAR.front+.04,0),p.a],
 ["sensorB",fp(p,-CAR.rear-.04,0),p.a+PI],
 ["sensorL",fp(p,1.4,-CAR.mirrors),p.a-PI/2],
 ["sensorR",fp(p,1.4,CAR.mirrors),p.a+PI/2]
 ];
 return arr.map(row=>({id:row[0],distance:sensorRay(row[1],row[2]),origin:row[1],angle:row[2]}));
}
function coachText(ds){
 if(state.won)return "Bagus! Perhatikan posisi roda dan jumlah koreksi sebelum mencoba tantangan berikutnya.";
 const smallest=ds.reduce((a,b)=>b.distance<a.distance?b:a,{distance:10,id:""});
 if(smallest.distance<.34)return "⚠ JARAK SANGAT DEKAT. Hentikan dan koreksi arah. Jangan paksa pedal.";
 if(smallest.distance<.68)return "⚠ Jarak sempit ("+fx(smallest.distance)+" m). Pantau spion dan sudut kendaraan.";
 if(state.intent<0){
 if(state.wheel>28)return "MUNDUR + SETIR KANAN → belakang ke kanan, DEPAN mengayun ke kiri. "+state.level.tip;
 if(state.wheel< -28)return "MUNDUR + SETIR KIRI → belakang ke kiri, DEPAN mengayun ke kanan. "+state.level.tip;
 return "MUNDUR LURUS: perhatikan dua spion dan posisi sudut depan. "+state.level.tip;
 }
 if(state.intent>0){
 if(state.wheel>28)return "MAJU + SETIR KANAN → depan menuju kanan, belakang memotong sisi dalam tikungan.";
 if(state.wheel< -28)return "MAJU + SETIR KIRI → depan menuju kiri, belakang memotong sisi dalam tikungan.";
 return "MAJU LURUS: koreksi setir sedikit demi sedikit dan awasi lebar sisa jalan.";
 }
 return state.level.tip;
}
function releaseWheel(){
 const id=state.wheelPointer;state.wheelPointer=null;state.wheelLastAngle=0;
 if(id!==null){try{if($("steering").hasPointerCapture(id))$("steering").releasePointerCapture(id);}catch(_){}}
}
function releaseAll(){
 releaseWheel();state.pedalPointers.clear();state.keys.clear();setIntent(0);updatePedals();
}
function setWheel(val){
 state.wheel=clamp(val,-CAR.steeringRange,CAR.steeringRange);
 $("rotor").style.transform="rotate("+state.wheel+"deg)";
 $("steerDirection").textContent=state.wheel>2?"KANAN":state.wheel< -2?"KIRI":"LURUS";
 $("steerValue").textContent=(state.wheel>0?"+":"")+Math.round(state.wheel)+"°";
 $("steerTurns").textContent=(Math.abs(state.wheel)/360).toFixed(2).replace(".",",")+"×";
 $("steering").setAttribute("aria-valuenow",Math.round(state.wheel));
 const pct=state.wheel/CAR.steeringRange*50,el=$("wheelProgress");
 el.style.left=(pct<0?50+pct:50)+"%";el.style.width=Math.abs(pct)+"%";
}
function updatePedals(){
 const sign=state.intent;
 $("forward").classList.toggle("pressed",sign>0);$("reverse").classList.toggle("pressed",sign<0);
 $("gearReadout").textContent=sign>0?"D":sign<0?"R":"N";
 $("movingLabel").textContent=sign>0?"MAJU":sign<0?"MUNDUR":"BERHENTI";
}
function setIntent(value){
 if(state.paused||state.won)value=0;
 if(value!==0&&state.lastDrive!==0&&state.lastDrive!==value)state.gearChanges++;
 if(value!==0)state.lastDrive=value;
 state.intent=value;
 updatePedals();
}
function syncIntent(){
 let plus=false,minus=false;
 for(const dir of state.pedalPointers.values()){if(dir===1)plus=true;else if(dir===-1)minus=true;}
 if(state.keys.has("ArrowUp")||state.keys.has("KeyW"))plus=true;
 if(state.keys.has("ArrowDown")||state.keys.has("KeyS"))minus=true;
 setIntent(plus===minus?0:plus?1:-1);
}
function reset(id){
 releaseAll();
 state.level=scenes.find(x=>x.id===id)||scenes[0];state.pose={...state.level.start};
 state.wheel=0;state.tyre=0;state.speed=0;state.elapsed=0;state.hits=0;state.gearChanges=0;state.lastDrive=0;state.lastImpact=-10;state.lastTime=performance.now();state.lastHud=0;
 state.won=false;state.paused=false;state.trail=[{...state.pose}];
 $("scenarioSelect").value=state.level.id;
 $("categoryLabel").textContent=state.level.group;
 $("missionName").textContent=state.level.name;
 $("difficultyLabel").textContent="●".repeat(state.level.difficulty)+"○".repeat(5-state.level.difficulty);
 $("widthInfo").textContent=state.level.roadLabel;
 $("missionHint").textContent=state.level.tip;
 $("completion").hidden=true;
 $("pauseBtn").textContent="⏸ JEDA";
 $("engineStatus").textContent="ENGINE V11 AKTIF";
 $("statusText").textContent="BEBAS BENTURAN • LATIHAN KECEPATAN RENDAH";
 setWheel(0);updatePedals();resize();draw();refreshHud(true);
 if(isBlocked(state.pose)){$("engineStatus").textContent="PERIKSA POSISI LEVEL";$("statusText").textContent="Posisi awal menyentuh objek: "+state.level.name;}
}
function finish(){
 state.won=true;state.speed=0;releaseAll();
 const rating=Math.max(1,3-Math.floor(state.hits/2)-Math.floor(state.gearChanges/6));
 $("completionStats").textContent=rating+" ★ • "+state.hits+" benturan • "+state.gearChanges+" pergantian arah • "+Math.floor(state.elapsed)+" dtk";
 $("completion").hidden=false;$("statusText").textContent="MISI BERHASIL — LANJUTKAN KE LEVEL BERIKUTNYA";
}
function goalReached(){
 const p=state.pose,g=state.level.goal;
 return Math.hypot(p.x-g.x,p.y-g.y)<.85&&Math.abs(wrap(p.a-g.a))<.22&&Math.abs(state.speed)<.5;
}
function resize(){
 const bounds=canvas.getBoundingClientRect();
 const w=Math.max(40,Math.round(bounds.width)),h=Math.max(40,Math.round(bounds.height));
 const dpr=Math.min(window.devicePixelRatio||1,2);
 if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}
 state.canvasW=w;state.canvasH=h;state.dpr=dpr;
}
function computeView(){
 const b=state.level.bounds,w=state.canvasW,h=state.canvasH;
 const fit=Math.max(2,Math.min((w-24)/b.w,(h-24)/b.h));
 let s=fit,cx=b.x+b.w/2,cy=b.y+b.h/2;
 if(state.camera==="follow"){s=Math.max(fit,Math.min(w/14,h/10));cx=state.pose.x+Math.cos(state.pose.a)*1.6;cy=state.pose.y+Math.sin(state.pose.a)*1.6;}
 if(state.camera==="rear"){s=Math.max(fit,Math.min(w/11,h/8));cx=state.pose.x-Math.cos(state.pose.a)*2;cy=state.pose.y-Math.sin(state.pose.a)*2;}
 state.view={s,ox:w/2-cx*s,oy:h/2-cy*s};
}
function renderPos(p){return{x:state.view.ox+p.x*state.view.s,y:state.view.oy+p.y*state.view.s};}
function rounded(x,y,w,h,r){
 r=Math.min(r,Math.abs(w)/2,Math.abs(h)/2);
 ctx.beginPath();
 if(ctx.roundRect)ctx.roundRect(x,y,w,h,r);
 else {ctx.moveTo(x+r,y);ctx.lineTo(x+w-r,y);ctx.quadraticCurveTo(x+w,y,x+w,y+r);ctx.lineTo(x+w,y+h-r);ctx.quadraticCurveTo(x+w,y+h,x+w-r,y+h);ctx.lineTo(x+r,y+h);ctx.quadraticCurveTo(x,y+h,x,y+h-r);ctx.quadraticCurveTo(x,y,x+r,y);ctx.closePath();}
}
function polyShape(poly,fill,stroke="",width=.055){
 ctx.beginPath();poly.forEach((p,i)=>i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y));ctx.closePath();
 if(fill){ctx.fillStyle=fill;ctx.fill();}
 if(stroke){ctx.strokeStyle=stroke;ctx.lineWidth=width;ctx.stroke();}
}
function drawWorld(){
 const b=state.level.bounds;
 ctx.fillStyle="#1d3a33";ctx.fillRect(b.x-.5,b.y-.5,b.w+1,b.h+1);
 for(let i=0;i<60;i++){const x=(i*13.47)%b.w,y=(i*7.41)%b.h;ctx.fillStyle="#2c554343";ctx.fillRect(x,y,.25,.13);}
 for(const r of state.level.pavement){
 ctx.fillStyle="#374957";ctx.fillRect(r.x,r.y,r.w,r.h);
 ctx.fillStyle="#435765";ctx.fillRect(r.x+.06,r.y+.06,r.w-.12,r.h-.12);
 ctx.lineWidth=.065;ctx.strokeStyle="#bdc6bd";ctx.strokeRect(r.x,r.y,r.w,r.h);
 }
 // Individual painted bays, curb lines and entrance lanes.
 for(const r of state.level.paint){
 ctx.fillStyle=r.style==="driveway"?"#53616c":"#4b5861";
 ctx.fillRect(r.x,r.y,r.w,r.h);
 ctx.save();ctx.strokeStyle="#e5dfb6";ctx.lineWidth=.095;ctx.setLineDash([.34,.19]);ctx.strokeRect(r.x+.10,r.y+.12,r.w-.20,r.h-.24);ctx.restore();
 }
 // dashed centre line for straight streets, with reduced opacity in parking spaces
 if(state.level.group==="GANG INDONESIA"||state.level.group==="JALAN & MANUVER"){
 const rr=state.level.pavement[0];ctx.beginPath();
 if(rr.w>rr.h*1.5){ctx.moveTo(rr.x+.35,rr.y+rr.h/2);ctx.lineTo(rr.x+rr.w-.35,rr.y+rr.h/2);}
 ctx.strokeStyle="#e5e2c438";ctx.lineWidth=.055;ctx.setLineDash([.55,.60]);ctx.stroke();ctx.setLineDash([]);
 }
 for(const m of state.level.labels||[]){
 ctx.save();ctx.font="900 "+Math.max(.38,Math.min(.65,state.level.bounds.w/38))+"px system-ui";ctx.textAlign="center";
 ctx.fillStyle="#e6eacccc";ctx.fillText(m.text,m.x,m.y);ctx.restore();
 }
}
function drawParkingGoal(){
 const g=state.level.goal;
 polyShape(vehicleHull(g),"#4cef9f25","#9dfbd5",.09);
 const p=fp(g,CAR.front+.57,0);ctx.fillStyle="#c9ffdf";ctx.font="900 .47px system-ui";ctx.textAlign="center";ctx.fillText("TARGET",p.x,p.y-.2);
}
function drawObstacle(o){
 if(o.type==="block"){
 const fill=o.style==="truck"?"#6e8394":"#988a70";
 polyShape(rectHull(o),fill,"#d0be9b",.07);
 ctx.save();ctx.strokeStyle="#f7dc9160";ctx.lineWidth=.06;
 for(let k=-1;k<4;k++){let x=o.x+k*.45;ctx.beginPath();ctx.moveTo(x,o.y);ctx.lineTo(x+.45,o.y+o.h);ctx.stroke();}
 ctx.restore();
 if(o.style==="truck"){ctx.fillStyle="#c4d2d4";ctx.fillRect(o.x+.18,o.y+.2,1.0,o.h-.4);ctx.fillStyle="#203c4c";ctx.fillRect(o.x+1.25,o.y+.2,.7,o.h-.4);}
 }else if(o.type==="car"){
 const pose={x:o.cx-Math.cos(o.a)*1.385,y:o.cy-Math.sin(o.a)*1.385,a:o.a};drawCar(pose,o.color,false);
 }else{
 ctx.save();ctx.translate(o.cx,o.cy);ctx.rotate(o.a);
 rounded(-.92,-.33,1.84,.66,.23);ctx.fillStyle="#273c49";ctx.fill();ctx.strokeStyle="#d2b36c";ctx.lineWidth=.07;ctx.stroke();
 ctx.fillStyle="#c87557";ctx.fillRect(-.42,-.23,.65,.46);ctx.fillStyle="#c7d9e1";ctx.fillRect(.58,-.28,.2,.56);
 for(const x of[-.66,.66]){ctx.fillStyle="#101a25";ctx.beginPath();ctx.arc(x,0,.28,0,TWO);ctx.fill();}
 ctx.restore();
 }
}
function drawCar(p,color,isPlayer){
 ctx.save();ctx.translate(p.x,p.y);ctx.rotate(p.a);
 ctx.shadowColor="#091722a9";ctx.shadowBlur=.22;ctx.shadowOffsetY=.09;
 rounded(-CAR.rear,-CAR.half,CAR.length,CAR.width,.30);
 ctx.fillStyle=color;ctx.fill();ctx.strokeStyle=isPlayer?"#ecffff":"#b4cad4";ctx.lineWidth=isPlayer?.095:.06;ctx.stroke();
 ctx.shadowBlur=0;ctx.shadowOffsetY=0;
 // Side pillars, doors, SUV silhouette and contrasting front/back.
 rounded(.02,-.71,2.68,1.42,.28);ctx.fillStyle=isPlayer?"#15577a":"#3e5f71";ctx.fill();
 rounded(2.40,-.67,.64,1.34,.12);ctx.fillStyle="#b3d9eb";ctx.fill();
 rounded(-.48,-.64,.43,1.28,.11);ctx.fillStyle="#243e54";ctx.fill();
 ctx.fillStyle=isPlayer?"#2a91ba":"#708595";ctx.fillRect(3.09,-.73,.38,1.46);
 // Windows and roof rail (car front to the right in local coordinates)
 ctx.strokeStyle="#e2f0f58a";ctx.lineWidth=.055;
 for(const v of[-.73,.73]){ctx.beginPath();ctx.moveTo(-.25,v);ctx.lineTo(2.40,v);ctx.stroke();}
 ctx.strokeStyle="#d2eaff";ctx.lineWidth=.12;ctx.beginPath();ctx.moveTo(3.43,-.48);ctx.lineTo(3.43,.48);ctx.stroke();
 // grille front at +x
 ctx.fillStyle="#102837";rounded(3.49,-.47,.10,.94,.05);ctx.fill();
 for(const yy of[-.29,-.14,0,.14,.29]){ctx.fillStyle="#a8cad9";ctx.fillRect(3.50,yy,.08,.045);}
 // red rear panel at -x
 ctx.fillStyle="#c73e43";ctx.fillRect(-.84,-.7,.14,1.4);
 // Light direction anchors
 for(const v of[-.70,.70]){
 ctx.fillStyle="#f7fbe3";ctx.fillRect(3.45,v-.12,.15,.24);
 ctx.fillStyle="#ff5966";ctx.fillRect(-.84,v-.14,.17,.28);
 }
 // wheels: front tyre steer, rear tyre fixed
 for(const v of[-CAR.half,CAR.half]){
 for(const u of[0,CAR.wheelbase]){
 ctx.save();ctx.translate(u,v);ctx.rotate(u>1?(isPlayer?rad(state.tyre):0):0);
 rounded(-.29,-.13,.58,.26,.08);ctx.fillStyle="#10131b";ctx.fill();ctx.restore();
 }
 // Mirror at front door — important for narrow gates.
 ctx.fillStyle="#d7f1fa";rounded(2.35,v>0?CAR.half:-(CAR.mirrors+.1),.4,.2,.05);ctx.fill();
 }
 if(isPlayer){
 // unmistakable front arrow + roof text and red rear marker
 ctx.fillStyle="#edffdd";ctx.textAlign="center";ctx.textBaseline="middle";ctx.font="1000 .53px system-ui";ctx.fillText("DEPAN",1.27,0);
 ctx.beginPath();ctx.moveTo(2.45,0);ctx.lineTo(2.06,-.23);ctx.lineTo(2.06,.23);ctx.closePath();ctx.fill();
 ctx.fillStyle="#ffd1c8";ctx.font="900 .30px system-ui";ctx.fillText("BLK",-0.24,0);
 }
 ctx.restore();
}
function drawTrail(){
 if(state.trail.length<2)return;
 ctx.beginPath();ctx.moveTo(state.trail[0].x,state.trail[0].y);
 for(const p of state.trail)ctx.lineTo(p.x,p.y);
 ctx.lineWidth=.07;ctx.strokeStyle="#8fc9ff88";ctx.stroke();
}
function drawPrediction(){
 if(!state.lines||state.won)return;
 const prefer=state.intent||((state.level.id.includes("ruko-nose")||state.level.id.includes("carport-back")||state.level.id.includes("mall")||state.level.id.includes("angle"))?-1:1);
 let p={...state.pose},points=[],front=[],rear=[];
 for(let i=0;i<135;i++){
 const n=nextPose(p,prefer*.9,state.tyre,.055);if(isBlocked(n))break;
 p=n;points.push(p);front.push(fp(p,CAR.front,-CAR.half));rear.push(fp(p,-CAR.rear,-CAR.half));
 }
 for(const [arr,col] of [[points,"#c8bdff9a"],[front,"#ffd08eaf"],[rear,"#83f1cfb0"]]){
 if(!arr.length)continue;ctx.save();ctx.beginPath();ctx.moveTo(arr[0].x,arr[0].y);
 for(const z of arr)ctx.lineTo(z.x,z.y);
 ctx.setLineDash([.14,.11]);ctx.lineWidth=.06;ctx.strokeStyle=col;ctx.stroke();ctx.restore();
 }
}
function drawCorners(){
 if(!state.corners)return;
 for(const [u,v,tint] of [[CAR.front,-CAR.half,"#ffcf8b"],[CAR.front,CAR.half,"#ffcf8b"],[-CAR.rear,-CAR.half,"#88efc3"],[-CAR.rear,CAR.half,"#88efc3"]]){
 const p=fp(state.pose,u,v);ctx.beginPath();ctx.arc(p.x,p.y,.13,0,TWO);ctx.fillStyle=tint;ctx.fill();ctx.lineWidth=.065;ctx.strokeStyle="#061621";ctx.stroke();
 }
}
function drawRadar(measures){
 if(!state.assist)return;
 for(const item of measures){
 if(item.distance>=1.1)continue;ctx.save();
 ctx.strokeStyle=item.distance<.4?"#ff7171a9":"#ffdc8b9c";ctx.lineWidth=.09;
 const p=item.origin,a=item.angle,d=item.distance;
 ctx.beginPath();ctx.moveTo(p.x,p.y);ctx.lineTo(p.x+d*Math.cos(a),p.y+d*Math.sin(a));ctx.stroke();ctx.restore();
 }
}
function draw(){
 const w=state.canvasW,h=state.canvasH;
 if(w<5||h<5)return;
 ctx.setTransform(state.dpr,0,0,state.dpr,0,0);ctx.clearRect(0,0,w,h);
 computeView();const v=state.view;
 ctx.save();ctx.translate(v.ox,v.oy);ctx.scale(v.s,v.s);
 drawWorld();drawParkingGoal();drawTrail();for(const o of state.level.obstacles)drawObstacle(o);
 drawPrediction();drawCar(state.pose,"#2491d0",true);drawCorners();drawRadar(state.lastMeasures||[]);ctx.restore();
 if(state.camera==="rear")drawRearGuide();
 if(state.paused){ctx.fillStyle="#081421ac";ctx.fillRect(0,0,w,h);ctx.font="950 23px system-ui";ctx.fillStyle="#eeffee";ctx.textAlign="center";ctx.fillText("GAME DIJEDA",w/2,h/2);}
}
function drawRearGuide(){
 const w=state.canvasW,h=state.canvasH;
 const bw=Math.min(200,w*.38),bh=Math.min(100,h*.30),x=w-bw-10,y=52;
 ctx.save();rounded(x,y,bw,bh,10);ctx.clip();ctx.fillStyle="#0b2333ef";ctx.fillRect(x,y,bw,bh);
 ctx.strokeStyle="#69edbd9b";ctx.lineWidth=2;ctx.setLineDash([5,5]);
 ctx.beginPath();ctx.moveTo(x+bw*.2,y+bh);ctx.lineTo(x+bw*.34,y+5);ctx.moveTo(x+bw*.8,y+bh);ctx.lineTo(x+bw*.66,y+5);ctx.stroke();ctx.setLineDash([]);
 ctx.fillStyle="#d8faff";ctx.textAlign="left";ctx.font="900 10px system-ui";ctx.fillText("PANDUAN MUNDUR 2D",x+8,y+16);
 ctx.fillStyle="#ff9a8b";ctx.fillRect(x+bw*.33,y+bh-8,bw*.34,4);
 ctx.restore();
}
function refreshHud(force=false){
 if(!force&&state.elapsed-state.lastHud<.12)return;state.lastHud=state.elapsed;
 const ms=measure();state.lastMeasures=ms;
 for(const item of ms){
 const el=$(item.id),d=item.distance;el.textContent=d>=5.45?"5,5m+":fx(d)+"m";el.className=d<.45?"danger":d<1?"warn":"";
 }
 $("speed").textContent=fx(Math.abs(state.speed)*3.6)+" km/j";
 $("tyreAngle").textContent=fx(state.tyre)+"°";
 $("collisions").textContent=String(state.hits);
 const ss=Math.floor(state.elapsed);$("timer").textContent=String(Math.floor(ss/60)).padStart(2,"0")+":"+String(ss%60).padStart(2,"0");
 $("changes").textContent=String(state.gearChanges);
 if(state.assist)$("coachMessage").textContent=coachText(ms);
}
function tick(timestamp){
 try {
 const dt=clamp((timestamp-state.lastTime)/1000||0,0,.04);
 state.lastTime=timestamp;
 if(!state.paused&&!state.won){
 state.elapsed+=dt;
 if(state.keys.has("ArrowLeft")||state.keys.has("KeyA"))setWheel(state.wheel-390*dt);
 if(state.keys.has("ArrowRight")||state.keys.has("KeyD"))setWheel(state.wheel+390*dt);
 const desired=state.wheel/CAR.steeringRange*CAR.maximumTyre;
 state.tyre+=(desired-state.tyre)*Math.min(1,7*dt);
 const reduction=1-.33*Math.abs(state.tyre)/CAR.maximumTyre;
 const target=state.intent*(state.mode==="precise"?1.05:1.9)*reduction;
 const rate=state.intent?2.1:3.3;
 state.speed+=clamp(target-state.speed,-rate*dt,rate*dt);
 if(Math.abs(state.speed)<.007)state.speed=0;
 if(Math.abs(state.speed)>.01){
 const steps=Math.max(1,Math.ceil(Math.abs(state.speed*dt)/.05));
 for(let i=0;i<steps;i++){
 const n=nextPose(state.pose,state.speed,state.tyre,dt/steps);
 if(isBlocked(n)){
 state.speed=0;if(state.elapsed-state.lastImpact>.72){state.hits++;state.lastImpact=state.elapsed;$("statusText").textContent="⚠ MENYENTUH BATAS / OBJEK — KOREKSI ARAH";try{navigator.vibrate&&navigator.vibrate(35)}catch(_){}}
 break;
 }
 state.pose=n;
 }
 const last=state.trail[state.trail.length-1];
 if(!last||Math.hypot(last.x-state.pose.x,last.y-state.pose.y)>.12){state.trail.push({...state.pose});if(state.trail.length>1800)state.trail.shift();}
 }
 if(goalReached())finish();
 }
 refreshHud();draw();
 }catch(error){if(!state.fatal){state.fatal=true;$("engineStatus").textContent="ENGINE ERROR";$("statusText").textContent=String(error.message||error);}}
 requestAnimationFrame(tick);
}
function angleFromPointer(e){
 const a=$("steering").getBoundingClientRect();
 return Math.atan2(e.clientY-a.top-a.height/2,e.clientX-a.left-a.width/2)*180/PI;
}
function attachSteering(){
 const wh=$("steering");wh.addEventListener("pointerdown",e=>{
 if(e.pointerType==="mouse"&&e.button!==0)return;
 if(e.target.closest("button"))return;
 e.preventDefault();if(state.wheelPointer!==null)releaseWheel();
 state.wheelPointer=e.pointerId;state.wheelLastAngle=angleFromPointer(e);
 try{wh.setPointerCapture(e.pointerId);}catch(_){}
 });
 wh.addEventListener("pointermove",e=>{
 if(state.wheelPointer!==e.pointerId)return;
 e.preventDefault();const now=angleFromPointer(e);let d=now-state.wheelLastAngle;
 if(d>180)d-=360;if(d< -180)d+=360;state.wheelLastAngle=now;setWheel(state.wheel+d);
 });
 function finishPointer(e){if(e.pointerId===state.wheelPointer)releaseWheel();}
 for(const event of["pointerup","pointercancel","lostpointercapture"])wh.addEventListener(event,finishPointer);
 for(const event of["pointerup","pointercancel"])window.addEventListener(event,finishPointer,true);
 wh.addEventListener("keydown",e=>{if(e.code==="ArrowLeft"){e.preventDefault();setWheel(state.wheel-30);}if(e.code==="ArrowRight"){e.preventDefault();setWheel(state.wheel+30);}if(e.code==="Home"){e.preventDefault();setWheel(0);}});
 $("centerWheel").addEventListener("pointerdown",e=>e.stopPropagation());
 $("centerWheel").addEventListener("click",e=>{e.stopPropagation();setWheel(0);});
}
function attachPedal(id,direction){
 const el=$(id);
 el.addEventListener("pointerdown",e=>{if(e.pointerType==="mouse"&&e.button!==0)return;e.preventDefault();if(state.paused||state.won)return;
 state.pedalPointers.set(e.pointerId,direction);try{el.setPointerCapture(e.pointerId);}catch(_){}syncIntent();});
 function stop(e){if(state.pedalPointers.has(e.pointerId)){state.pedalPointers.delete(e.pointerId);syncIntent();}}
 for(const ev of["pointerup","pointercancel","lostpointercapture"])el.addEventListener(ev,stop);
 for(const ev of["pointerup","pointercancel"])window.addEventListener(ev,stop,true);
 el.addEventListener("contextmenu",e=>e.preventDefault());
}
function makeMenus(){
 const sel=$("scenarioSelect");sel.innerHTML="";
 const groups=[...new Set(scenes.map(x=>x.group))];
 for(const group of groups){
 const wrap=document.createElement("optgroup");wrap.label=group;
 for(const l of scenes.filter(x=>x.group===group)){
 const el=document.createElement("option");el.value=l.id;el.textContent=l.name;wrap.appendChild(el);
 }
 sel.appendChild(wrap);
 }
 sel.addEventListener("change",()=>reset(sel.value));
}
function nextScene(){
 const i=scenes.findIndex(x=>x.id===state.level.id);reset(scenes[(i+1)%scenes.length].id);
}
function cycleCamera(){
 state.camera=state.camera==="map"?"follow":state.camera==="follow"?"rear":"map";
 const labels={map:"▣ PETA",follow:"◎ IKUTI",rear:"▤ MUNDUR"};
 const detail={map:"PETA LENGKAP",follow:"KAMERA IKUTI",rear:"KAMERA MUNDUR 2D"};
 $("cameraBtn").textContent=labels[state.camera];$("viewLabel").textContent=detail[state.camera];
}
function homeView(){state.camera="map";$("cameraBtn").textContent="▣ PETA";$("viewLabel").textContent="PETA LENGKAP";resize();draw();}
async function fullscreen(){
 const active=!!document.fullscreenElement||document.body.classList.contains("pseudo-full");
 if(active){
 document.body.classList.remove("pseudo-full");
 try{if(document.fullscreenElement)await document.exitFullscreen();}catch(_){}
 $("fullBtn").textContent="⛶ FULLSCREEN";resize();return;
 }
 let ok=false;try{if(document.documentElement.requestFullscreen){await document.documentElement.requestFullscreen();ok=!!document.fullscreenElement;}}catch(_){}
 if(!ok)document.body.classList.add("pseudo-full");
 $("fullBtn").textContent="✕ KELUAR";
 if(ok){try{if(screen.orientation&&screen.orientation.lock)await screen.orientation.lock("landscape");}catch(_){}}
 resize();
}
function bindUI(){
 attachSteering();attachPedal("forward",1);attachPedal("reverse",-1);makeMenus();
 $("resetBtn").addEventListener("click",()=>reset(state.level.id));
 $("retryBtn").addEventListener("click",()=>reset(state.level.id));
 $("nextBtn").addEventListener("click",nextScene);
 $("cameraBtn").addEventListener("click",cycleCamera);
 $("homeView").addEventListener("click",homeView);
 $("modeBtn").addEventListener("click",()=>{state.mode=state.mode==="precise"?"normal":"precise";$("modeBtn").textContent=state.mode==="precise"?"◉ PRESISI":"⚡ NORMAL";});
 $("assistBtn").addEventListener("click",()=>{state.assist=!state.assist;$("assistBtn").textContent=state.assist?"✦ PANDUAN ON":"✦ PANDUAN OFF";$("assistBtn").classList.toggle("active",state.assist);$("assistBtn").setAttribute("aria-pressed",state.assist);$("coachMessage").hidden=!state.assist;});
 $("lineBtn").addEventListener("click",()=>{state.lines=!state.lines;$("lineBtn").textContent=state.lines?"JALUR ✓":"JALUR —";});
 $("cornerBtn").addEventListener("click",()=>{state.corners=!state.corners;$("cornerBtn").textContent=state.corners?"SUDUT ✓":"SUDUT —";});
 $("pauseBtn").addEventListener("click",()=>{state.paused=!state.paused;if(state.paused)releaseAll();$("pauseBtn").textContent=state.paused?"▶ LANJUT":"⏸ JEDA";});
 $("fullBtn").addEventListener("click",fullscreen);
 document.addEventListener("fullscreenchange",()=>{$("fullBtn").textContent=document.fullscreenElement||document.body.classList.contains("pseudo-full")?"✕ KELUAR":"⛶ FULLSCREEN";resize();});
 window.addEventListener("resize",resize);window.addEventListener("orientationchange",resize);
 window.addEventListener("blur",releaseAll);document.addEventListener("visibilitychange",()=>{if(document.hidden)releaseAll();});
 document.addEventListener("keydown",e=>{
 if(e.target.tagName==="SELECT")return;
 if(["ArrowUp","ArrowDown","ArrowLeft","ArrowRight","KeyW","KeyS","KeyA","KeyD","Space","KeyR"].includes(e.code))e.preventDefault();
 if(["ArrowUp","ArrowDown","ArrowLeft","ArrowRight","KeyW","KeyS","KeyA","KeyD"].includes(e.code)){state.keys.add(e.code);syncIntent();}
 if(e.code==="Space")setWheel(0);
 if(e.code==="KeyR"&&!e.repeat)reset(state.level.id);
 if(e.key==="Escape"&&document.body.classList.contains("pseudo-full")){document.body.classList.remove("pseudo-full");$("fullBtn").textContent="⛶ FULLSCREEN";}
 });
 document.addEventListener("keyup",e=>{state.keys.delete(e.code);syncIntent();});
 // Stop pinch and double-tap zoom in the gaming surface on Safari.
 for(const type of["gesturestart","gesturechange","gestureend"])document.addEventListener(type,e=>{if(e.cancelable)e.preventDefault();},{passive:false});
 document.addEventListener("dblclick",e=>{if(e.cancelable&&e.target.closest(".steering,.pedal,.viewport"))e.preventDefault();},{passive:false});
 const rObserver=typeof ResizeObserver!=="undefined"?new ResizeObserver(resize):null;
 if(rObserver)rObserver.observe(canvas);
}
window.driveLabTest={
 get:()=>({level:state.level.id,pose:{...state.pose},blocked:isBlocked(state.pose),wheel:state.wheel,tyre:state.tyre,speed:state.speed,intent:state.intent,collisions:state.hits,gearChanges:state.gearChanges,goalBlocked:isBlocked(state.level.goal),won:state.won,frame:{w:state.canvasW,h:state.canvasH}}),
 levels:()=>scenes.map(x=>x.id),reset,steer:setWheel,blocked:isBlocked,
 press:dir=>{state.pedalPointers.set(-999,dir);syncIntent();},release:()=>{state.pedalPointers.delete(-999);syncIntent();},
 move:(v,secs)=>{let p={...state.pose};for(let t=0;t<secs;t+=.015){const q=nextPose(p,v,state.tyre,.015);if(isBlocked(q))return false;p=q;}state.pose=p;draw();return true;},
 step:(dt,velocity,tyre)=>{const next=nextPose(state.pose,velocity,tyre,dt);if(isBlocked(next))return false;state.pose=next;draw();return true;}
};
bindUI();reset(scenes[0].id);
$("engineStatus").textContent="ENGINE V11 AKTIF";
requestAnimationFrame(tick);
})();
