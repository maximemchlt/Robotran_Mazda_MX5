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
#	==> Generation Date: Thu Mar 26 15:25:15 2026
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Robotran_Mazda_MX5
#
#	==> Number of joints: 18
#
#	==> Function: F7 - Link Forces (1D)
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos, sqrt

def link(frc, trq, Flink, Z, Zd, s, tsim):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S7 = sin(q[7])
    C7 = cos(q[7])
    S10 = sin(q[10])
    C10 = cos(q[10])
 
# Augmented Joint Position Vectors

 
# Link anchor points Kinematics

    RLlnk2_22 = s.dpt[2,9]*C7-s.dpt[3,9]*S7
    RLlnk2_32 = s.dpt[2,9]*S7+s.dpt[3,9]*C7
    POlnk2_22 = RLlnk2_22+s.dpt[2,1]
    POlnk2_32 = RLlnk2_32+s.dpt[3,1]
    ORlnk2_22 = -qd[7]*RLlnk2_32
    ORlnk2_32 = qd[7]*RLlnk2_22
    Plnk11 = s.dpt[1,1]-s.dpt[1,3]
    Plnk21 = POlnk2_22-s.dpt[2,3]
    Plnk31 = POlnk2_32-s.dpt[3,3]
    PPlnk1 = Plnk11*Plnk11+Plnk21*Plnk21+Plnk31*Plnk31
    Z1 = sqrt(PPlnk1)
    e11 = Plnk11/Z1
    e21 = Plnk21/Z1
    e31 = Plnk31/Z1
    Zd1 = ORlnk2_22*e21+ORlnk2_32*e31
    RLlnk3_22 = s.dpt[2,14]*C10-s.dpt[3,14]*S10
    RLlnk3_32 = s.dpt[2,14]*S10+s.dpt[3,14]*C10
    POlnk3_22 = RLlnk3_22+s.dpt[2,4]
    POlnk3_32 = RLlnk3_32+s.dpt[3,4]
    ORlnk3_22 = -qd[10]*RLlnk3_32
    ORlnk3_32 = qd[10]*RLlnk3_22
    Plnk12 = -s.dpt[1,4]+s.dpt[1,6]
    Plnk22 = -POlnk3_22+s.dpt[2,6]
    Plnk32 = -POlnk3_32+s.dpt[3,6]
    PPlnk2 = Plnk12*Plnk12+Plnk22*Plnk22+Plnk32*Plnk32
    Z2 = sqrt(PPlnk2)
    e12 = Plnk12/Z2
    e22 = Plnk22/Z2
    e32 = Plnk32/Z2
    Zd2 = -ORlnk3_22*e22-ORlnk3_32*e32

# Link Forces 

    Flink1 = s.user_LinkForces(Z1,Zd1,s,tsim,1)
    Flink2 = s.user_LinkForces(Z2,Zd2,s,tsim,2)
 
# Link Dynamics: forces projection on body-fixed frames

    fPlnk11 = Flink1*e11
    fPlnk21 = Flink1*e21
    fPlnk31 = Flink1*e31
    trqlnk6_1_1 = -fPlnk21*(s.dpt[3,3]-s.l[3,6])+fPlnk31*s.dpt[2,3]
    trqlnk6_1_2 = fPlnk11*(s.dpt[3,3]-s.l[3,6])-fPlnk31*s.dpt[1,3]
    trqlnk6_1_3 = -fPlnk11*s.dpt[2,3]+fPlnk21*s.dpt[1,3]
    fSlnk11 = Flink1*e11
    fSlnk21 = Flink1*(e21*C7+e31*S7)
    fSlnk31 = Flink1*(-e21*S7+e31*C7)
    trqlnk7_1_1 = fSlnk21*s.dpt[3,9]-fSlnk31*(s.dpt[2,9]-s.l[2,7])
    trqlnk7_1_2 = -fSlnk11*s.dpt[3,9]
    trqlnk7_1_3 = fSlnk11*(s.dpt[2,9]-s.l[2,7])
    fPlnk12 = Flink2*e12
    fPlnk22 = Flink2*(e22*C10+e32*S10)
    fPlnk32 = Flink2*(-e22*S10+e32*C10)
    trqlnk10_2_1 = -fPlnk22*s.dpt[3,14]+fPlnk32*(s.dpt[2,14]-s.l[2,10])
    trqlnk10_2_2 = fPlnk12*s.dpt[3,14]
    trqlnk10_2_3 = -fPlnk12*(s.dpt[2,14]-s.l[2,10])
    fSlnk12 = Flink2*e12
    fSlnk22 = Flink2*e22
    fSlnk32 = Flink2*e32
    frclnk6_2_1 = fPlnk11-fSlnk12
    frclnk6_2_2 = fPlnk21-fSlnk22
    frclnk6_2_3 = fPlnk31-fSlnk32
    trqlnk6_2_1 = trqlnk6_1_1+fSlnk22*(s.dpt[3,6]-s.l[3,6])-fSlnk32*s.dpt[2,6]
    trqlnk6_2_2 = trqlnk6_1_2-fSlnk12*(s.dpt[3,6]-s.l[3,6])+fSlnk32*s.dpt[1,6]
    trqlnk6_2_3 = trqlnk6_1_3+fSlnk12*s.dpt[2,6]-fSlnk22*s.dpt[1,6]
 
# Symbolic model output

    frc[1,6] = s.frc[1,6]+frclnk6_2_1
    frc[2,6] = s.frc[2,6]+frclnk6_2_2
    frc[3,6] = s.frc[3,6]+frclnk6_2_3
    trq[1,6] = s.trq[1,6]+trqlnk6_2_1
    trq[2,6] = s.trq[2,6]+trqlnk6_2_2
    trq[3,6] = s.trq[3,6]+trqlnk6_2_3
    frc[1,7] = s.frc[1,7]-fSlnk11
    frc[2,7] = s.frc[2,7]-fSlnk21
    frc[3,7] = s.frc[3,7]-fSlnk31
    trq[1,7] = s.trq[1,7]+trqlnk7_1_1
    trq[2,7] = s.trq[2,7]+trqlnk7_1_2
    trq[3,7] = s.trq[3,7]+trqlnk7_1_3
    frc[1,10] = s.frc[1,10]+fPlnk12
    frc[2,10] = s.frc[2,10]+fPlnk22
    frc[3,10] = s.frc[3,10]+fPlnk32
    trq[1,10] = s.trq[1,10]+trqlnk10_2_1
    trq[2,10] = s.trq[2,10]+trqlnk10_2_2
    trq[3,10] = s.trq[3,10]+trqlnk10_2_3
 
# Symbolic model output

    Z[1] = Z1
    Zd[1] = Zd1
    Flink[1] = Flink1
    Z[2] = Z2
    Zd[2] = Zd2
    Flink[2] = Flink2

# Number of continuation lines = 0


