#
#	MBsysTran - Release 8.1
#
#	Copyright 
#	Universite catholique de Louvain (UCLouvain) 
#	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
#	2, Place du Levant
#	1348 Louvain-la-Neuve 
#	Belgium 
#
#	http://www.robotran.be 
#
#	==> Generation Date: Thu Mar 26 12:39:45 2026
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Robotran_Mazda_MX5
#
#	==> Number of joints: 12
#
#	==> Function: F8 - Constraints and Constraints Jacobian(h, J)
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos

def cons_hJ(h, Jac, s):
    q = s.q
 
# Trigonometric functions

    S7 = sin(q[7])
    C7 = cos(q[7])
    S8 = sin(q[8])
    C8 = cos(q[8])
    S9 = sin(q[9])
    C9 = cos(q[9])
    S11 = sin(q[11])
    C11 = cos(q[11])
    S12 = sin(q[12])
    C12 = cos(q[12])
    S10 = sin(q[10])
    C10 = cos(q[10])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

    RLlp1_22 = s.dpt[2,7]*C7
    RLlp1_32 = s.dpt[2,7]*S7
    POlp1_22 = RLlp1_22+s.dpt[2,1]
    POlp1_32 = RLlp1_32+s.dpt[3,1]
    ROlp2_82 = -C8*S9-S8*C9
    ROlp2_92 = C8*C9-S8*S9
    RLlp2_22 = s.dpt[2,9]*C8
    RLlp2_32 = s.dpt[2,9]*S8
    POlp2_22 = RLlp2_22+s.dpt[2,2]
    POlp2_32 = RLlp2_32+s.dpt[3,2]
    RLlp2_23 = ROlp2_82*s.dpt[3,10]
    RLlp2_33 = ROlp2_92*s.dpt[3,10]
    POlp2_23 = POlp2_22+RLlp2_23
    POlp2_33 = POlp2_32+RLlp2_33
    JTlp2_23_1 = -RLlp2_32-RLlp2_33
    JTlp2_33_1 = RLlp2_22+RLlp2_23
    h_2 = POlp1_22-POlp2_23
    h_3 = POlp1_32-POlp2_33
    ROlp3_82 = -C11*S12-S11*C12
    ROlp3_92 = C11*C12-S11*S12
    RLlp3_22 = s.dpt[2,13]*C11
    RLlp3_32 = s.dpt[2,13]*S11
    POlp3_22 = RLlp3_22+s.dpt[2,5]
    POlp3_32 = RLlp3_32+s.dpt[3,5]
    RLlp3_23 = ROlp3_82*s.dpt[3,14]
    RLlp3_33 = ROlp3_92*s.dpt[3,14]
    POlp3_23 = POlp3_22+RLlp3_23
    POlp3_33 = POlp3_32+RLlp3_33
    JTlp3_23_1 = -RLlp3_32-RLlp3_33
    JTlp3_33_1 = RLlp3_22+RLlp3_23
    RLlp4_22 = s.dpt[2,11]*C10
    RLlp4_32 = s.dpt[2,11]*S10
    POlp4_22 = RLlp4_22+s.dpt[2,4]
    POlp4_32 = RLlp4_32+s.dpt[3,4]
    h_5 = POlp3_23-POlp4_22
    h_6 = POlp3_33-POlp4_32
    h[1] = h_2
    h[2] = h_3
    h[3] = h_5
    h[4] = h_6
    Jac[1,1] = 0
    Jac[1,2] = 0
    Jac[1,3] = 0
    Jac[1,4] = 0
    Jac[1,5] = 0
    Jac[1,6] = 0
    Jac[1,7] = -RLlp1_32
    Jac[1,8] = -JTlp2_23_1
    Jac[1,9] = RLlp2_33
    Jac[1,10] = 0
    Jac[1,11] = 0
    Jac[1,12] = 0
    Jac[2,1] = 0
    Jac[2,2] = 0
    Jac[2,3] = 0
    Jac[2,4] = 0
    Jac[2,5] = 0
    Jac[2,6] = 0
    Jac[2,7] = RLlp1_22
    Jac[2,8] = -JTlp2_33_1
    Jac[2,9] = -RLlp2_23
    Jac[2,10] = 0
    Jac[2,11] = 0
    Jac[2,12] = 0
    Jac[3,1] = 0
    Jac[3,2] = 0
    Jac[3,3] = 0
    Jac[3,4] = 0
    Jac[3,5] = 0
    Jac[3,6] = 0
    Jac[3,7] = 0
    Jac[3,8] = 0
    Jac[3,9] = 0
    Jac[3,10] = RLlp4_32
    Jac[3,11] = JTlp3_23_1
    Jac[3,12] = -RLlp3_33
    Jac[4,1] = 0
    Jac[4,2] = 0
    Jac[4,3] = 0
    Jac[4,4] = 0
    Jac[4,5] = 0
    Jac[4,6] = 0
    Jac[4,7] = 0
    Jac[4,8] = 0
    Jac[4,9] = 0
    Jac[4,10] = -RLlp4_22
    Jac[4,11] = JTlp3_33_1
    Jac[4,12] = RLlp3_23

# Number of continuation lines = 0


