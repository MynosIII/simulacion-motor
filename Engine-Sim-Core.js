/* JavaScript adaptation of engine-sim, Copyright 2022 AngeTheGreat
 * (Ange Yaghi), MIT. Used by the drawing of the engine view; the simulation
 * itself is in Engine-Sim-Physics.js.
 */
function esPlaceCylinder(pinX,pinY,bankX,bankY,dx,dy,rodLength){
  // PistonEngineSimulator::placeCylinder: same line/circle intersection.
  let a=dx*dx+dy*dy,b=-2*dx*(pinX-bankX)-2*dy*(pinY-bankY),c=(pinX-bankX)**2+(pinY-bankY)**2-rodLength**2,det=b*b-4*a*c;if(det<0)return null;let s=Math.max((-b+Math.sqrt(det))/(2*a),(-b-Math.sqrt(det))/(2*a));if(s<0)return null;return {x:bankX+s*dx,y:bankY+s*dy};
}

/*
Copyright 2022 AngeTheGreat (Ange Yaghi)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/
