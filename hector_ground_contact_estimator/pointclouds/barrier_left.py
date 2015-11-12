#!/usr/bin/python

from random import randint
# Open a file
fo = open("pc_barrier_left_close.pcd", "wb")

sizex=150
sizey=150
x = list()
y = list()
z = list()
for xi in range(sizex):
    for yi in range(sizey):  
        x.append(xi/25.0)
        y.append(yi/25.0)
        if((xi<=50) and (yi>25) and (yi<=50)):
            z.append(0.5)
        else:
            z.append(0.0)
            
            
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
