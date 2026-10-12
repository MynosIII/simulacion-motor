/* JavaScript adaptations of engine-sim, Copyright 2022 AngeTheGreat
 * (Ange Yaghi), MIT. See embedded license and source mapping in the HTML.
 * These are selected algorithms, not the complete C++ physics engine.
 */
class ESFunction {
  constructor(samples,radius){this.samples=samples.sort((a,b)=>a[0]-b[0]);this.radius=radius}
  sampleTriangle(x){const s=this.samples;if(!s.length)return 0;if(x<=s[0][0])return s[0][1];if(x>=s.at(-1)[0])return s.at(-1)[1];let sum=0,weight=0;for(const [sx,sy] of s){if(Math.abs(sx-x)>this.radius)continue;let w=(this.radius-Math.abs(sx-x))/this.radius;sum+=w*sy;weight+=w}return weight?sum/weight:0}
}
function esHarmonicLobe(durationDeg,liftMm,gamma=1.1,steps=100){
  // GenerateHarmonicCamLobeNode::_evaluate, scripting/include/actions.h
  let angle=durationDeg*Math.PI/180/4,s=Math.pow(2*1.27/liftMm,1/gamma)-1,k=Math.acos(s)/angle,extents=Math.PI/k,step=extents/(steps-5),samples=[[0,liftMm]];
  for(let i=1;i<steps;i++){let x=i*step,lift=x>=extents?0:liftMm*Math.pow(.5+.5*Math.cos(k*x),gamma);samples.push([x,lift],[-x,lift])}return new ESFunction(samples,step);
}
class ESCamshaft {
  constructor(profile){this.profile=profile}
  valveLift(crankAngle,centerDeg){let theta=(crankAngle-centerDeg*Math.PI/180)*.5;theta=((theta%(2*Math.PI))+2*Math.PI)%(2*Math.PI);if(theta>=Math.PI)theta-=2*Math.PI;return this.profile.sampleTriangle(theta)}
}
function esPlaceCylinder(pinX,pinY,bankX,bankY,dx,dy,rodLength){
  // PistonEngineSimulator::placeCylinder: same line/circle intersection.
  let a=dx*dx+dy*dy,b=-2*dx*(pinX-bankX)-2*dy*(pinY-bankY),c=(pinX-bankX)**2+(pinY-bankY)**2-rodLength**2,det=b*b-4*a*c;if(det<0)return null;let s=Math.max((-b+Math.sqrt(det))/(2*a),(-b-Math.sqrt(det))/(2*a));if(s<0)return null;return {x:bankX+s*dx,y:bankY+s*dy};
}
class ESIgnition {
  constructor(){this.angle=0;this.events=0;this.plugs=[];this.limitTimer=0}
  configure(order){this.plugs=order.map((c,i)=>({cylinder:c-1,angle:2*Math.PI+i*4*Math.PI/order.length}));this.events=0;this.angle=0}
  advance(delta,advanceDeg,enabled,rpm,redline,dt){let from=this.angle,to=from+delta,result=[];if(enabled&&this.limitTimer===0){for(const plug of this.plugs){let adjusted=plug.angle-advanceDeg*Math.PI/180,k=Math.floor((from-adjusted)/(4*Math.PI))+1;for(let event=adjusted+k*4*Math.PI;event<=to;event+=4*Math.PI)result.push(plug.cylinder)}}this.limitTimer=Math.max(0,this.limitTimer-dt);if(Math.abs(rpm)>redline)this.limitTimer=.1;this.angle=to;this.events+=result.length;return result}
}

/*
Copyright 2022 AngeTheGreat (Ange Yaghi)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/
