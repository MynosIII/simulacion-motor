import struct,sys,json
d=open(sys.argv[1],'rb').read()
n=struct.unpack_from('<i',d,0)[0];o=4;out={}
for k in range(n):
    name=d[o:o+64].split(b'\0')[0].decode();mat=d[o+64:o+128].split(b'\0')[0].decode();o2=o+128
    mi,pi,pii,si,ot=struct.unpack_from('<5i',d,o2);o2+=20
    fl=struct.unpack_from('<19f',d,o2);o2+=76
    uv,nv,nf,nb,mb,flags,vds=struct.unpack_from('<5iIi',d,o2);o2+=28
    print(name,mat,'type',ot,'parent',pi,'pos',fl[0:3],'euler',fl[3:6],'scale',fl[10:13],'min',fl[13:16],'max',fl[16:19],'uv',uv,'nv',nv,'nf',nf,'nb',nb,'flags',flags,'vds',vds, 'stride', vds/nv if nv else 0)
    o=o2
    if ot==0 or vds>0:
        stride=vds//nv
        verts=[struct.unpack_from('<%df'%(stride//4),d,o+i*stride) for i in range(nv)]
        o+=vds
        idx=struct.unpack_from('<%dH'%(nf*3),d,o);o+=nf*6
        o+=nb*4
        print('  v0',verts[0]);print('  v1',verts[1])
        out[name]={'v':[(v[0],v[1],v[2]) for v in verts],'i':idx}
json.dump(out,open(sys.argv[2],'w'))
print('end',o,len(d))
