#!/usr/bin/python

# Open a file
fo = open("zwei_ebenen_steigend.pcd", "wb")
sizex=50
sizey=50
x = list()
y = list()
z = list()
for xi in range(sizex):
    for yi in range(sizey):
        x.append(-xi/25.0)
        y.append(-yi/25.0)
        if(xi>=25.0):
            z.append(0.3+(xi*1.0/100.0))
        else:
            z.append((xi*1.0/70.0))
            
            
fo.write( bytes("\n#.PCD v0.7 - Point Cloud Data file format \nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n", 'UTF-8'))
fo.write( bytes("TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH ", 'UTF-8'))
fo.write( bytes(str(sizex*sizey), 'UTF-8'))
fo.write( bytes("\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS ", 'UTF-8'))
fo.write( bytes(str(sizex*sizey), 'UTF-8'))
fo.write( bytes("\nDATA ascii\n", 'UTF-8'))
        
for i in range(sizex*sizey):
    fo.write( bytes(str(x[i])+" ", 'UTF-8'))
    fo.write( bytes(str(y[i])+" ", 'UTF-8'))
    fo.write( bytes(str(z[i])+" ", 'UTF-8'))
    fo.write( bytes(str(0)+"\n", 'UTF-8'))
    
        


# Close opend file
fo.close()
