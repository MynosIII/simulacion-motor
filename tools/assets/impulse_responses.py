import wave,struct,base64,json,sys
lib={'default_0':('smooth/smooth_39.wav',0.001),'mild_exhaust_0':('new/mild_exhaust.wav',0.01),'mild_exhaust_0_reverb':('new/mild_exhaust_reverb.wav',0.01),'minimal_muffling_02':('new/minimal_muffling_02.wav',0.01)}
out={}
for k,(f,v) in lib.items():
    w=wave.open(sys.argv[1]+'/'+f);n=w.getnframes();d=struct.unpack('<%dh'%n,w.readframes(n))
    clip=0
    for i,x in enumerate(d):
        if abs(x)>100: clip=i+1
    m=min(10000,clip)
    out[k]={'volume':v,'samples':base64.b64encode(struct.pack('<%dh'%m,*d[:m])).decode()}
    print(k,n,clip,m)
json.dump(out,open(sys.argv[2],'w'))
