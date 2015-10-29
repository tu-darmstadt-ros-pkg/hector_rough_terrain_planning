#!/usr/bin/python
from random import randint
# Open a file
fo = open("stepfield01.pcd", "wb")
sizex=20
sizey=20
x = list()
y = list()
z = list()
for xi in range(sizex):
    for yi in range(sizey):        
        r=randint(0,99)
        err=r/10000.0
        for xi_small in range (5):
            for yi_small in range (5):                        
                x.append(-xi/10.0-xi_small/50.0)
                y.append(-yi/10.0-yi_small/50.0)
                z.append(r/200.0)
                
            
            
fo.write( bytes("\n#.PCD v0.7 - Point Cloud Data file format \nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n", 'UTF-8'))
fo.write( bytes("TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH ", 'UTF-8'))
fo.write( bytes(str(25*sizex*sizey), 'UTF-8'))
fo.write( bytes("\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS ", 'UTF-8'))
fo.write( bytes(str(25*sizex*sizey), 'UTF-8'))
fo.write( bytes("\nDATA ascii\n", 'UTF-8'))
        
for i in range(25*sizex*sizey):
    fo.write( bytes(str(x[i])+" ", 'UTF-8'))
    fo.write( bytes(str(y[i])+" ", 'UTF-8'))
    fo.write( bytes(str(z[i])+" ", 'UTF-8'))
    fo.write( bytes(str(0)+"\n", 'UTF-8'))
    
        


# Close opend file
fo.close()
