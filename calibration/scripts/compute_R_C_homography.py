import numpy as np
from dlt import DLTcalib

R_coords = []
C_coords = []
with open('sample_R_C_hom.txt', 'r') as f:
	lines = f.readlines()
	for line in lines:
		tmp = line.split(' ')
		R_coords.append([np.float32(tmp[0]),np.float32(tmp[1]),np.float32(tmp[2])-0.1])
		C_coords.append([np.float32(tmp[3]),np.float32(tmp[4])])
R_coords = np.array(R_coords)
C_coords = np.array(C_coords)
#print(R_coords)
#print(C_coords)
P, err = DLTcalib(3, R_coords, C_coords)
print(P)
print(err)
np.savetxt('R_C_Hom.out',P)
