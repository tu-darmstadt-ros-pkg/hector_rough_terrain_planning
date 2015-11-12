#!/usr/bin/python

from random import randint
# Open a file
fo = open("pc_dach_global.pcd", "wb")
sizex=150
sizey=150
x = list()
y = list()
z = list()
for xi in range(sizex):
    for yi in range(sizey):  
        r=randint(0,99)
        err=r/10000.0
        x.append(xi/25.0)
        y.append(yi/25.0)
        z.append(-abs(xi-75.0)/100.0)
            
            
fo.write("\n#.PCD v0.7 - Point Cloud Data file format \nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n")
fo.write( "TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH ")
fo.write( str(sizex*sizey))
fo.write( "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS ")
fo.write( str(sizex*sizey))
fo.write( "\nDATA ascii\n")
        
for i in range(sizex*sizey):
    fo.write( str(x[i])+" ")
    fo.write( str(y[i])+" ")
    fo.write( str(z[i])+" ")
    fo.write( str(0)+"\n")
    
        


# Close opend file
fo.close()
