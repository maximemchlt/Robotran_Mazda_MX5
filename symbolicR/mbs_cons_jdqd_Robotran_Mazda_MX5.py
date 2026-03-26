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
#	==> Function: F18 - Constraints Quadratic Velocity Terms (Jdqd)
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos

def cons_jdqd(Jdqd, s):
    q = s.q
    qd = s.qd
 
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

 
# Constraints Quadratic Terms

    RLjdqd1_22 = s.dpt[2,7]*C7
    RLjdqd1_32 = s.dpt[2,7]*S7
    ORjdqd1_22 = -RLjdqd1_32*qd[7]
    ORjdqd1_32 = RLjdqd1_22*qd[7]
    Apqpjdqd1_22 = -ORjdqd1_32*qd[7]
    Apqpjdqd1_32 = ORjdqd1_22*qd[7]
    ROjdqd2_82 = -C8*S9-S8*C9
    ROjdqd2_92 = C8*C9-S8*S9
    RLjdqd2_22 = s.dpt[2,9]*C8
    RLjdqd2_32 = s.dpt[2,9]*S8
    OMjdqd2_12 = qd[8]+qd[9]
    ORjdqd2_22 = -RLjdqd2_32*qd[8]
    ORjdqd2_32 = RLjdqd2_22*qd[8]
    Apqpjdqd2_22 = -ORjdqd2_32*qd[8]
    Apqpjdqd2_32 = ORjdqd2_22*qd[8]
    RLjdqd2_23 = ROjdqd2_82*s.dpt[3,10]
    RLjdqd2_33 = ROjdqd2_92*s.dpt[3,10]
    ORjdqd2_23 = -OMjdqd2_12*RLjdqd2_33
    ORjdqd2_33 = OMjdqd2_12*RLjdqd2_23
    Apqpjdqd2_23 = Apqpjdqd2_22-OMjdqd2_12*ORjdqd2_33
    Apqpjdqd2_33 = Apqpjdqd2_32+OMjdqd2_12*ORjdqd2_23
    jdqd2 = Apqpjdqd1_22-Apqpjdqd2_23
    jdqd3 = Apqpjdqd1_32-Apqpjdqd2_33
    ROjdqd3_82 = -C11*S12-S11*C12
    ROjdqd3_92 = C11*C12-S11*S12
    RLjdqd3_22 = s.dpt[2,13]*C11
    RLjdqd3_32 = s.dpt[2,13]*S11
    OMjdqd3_12 = qd[11]+qd[12]
    ORjdqd3_22 = -RLjdqd3_32*qd[11]
    ORjdqd3_32 = RLjdqd3_22*qd[11]
    Apqpjdqd3_22 = -ORjdqd3_32*qd[11]
    Apqpjdqd3_32 = ORjdqd3_22*qd[11]
    RLjdqd3_23 = ROjdqd3_82*s.dpt[3,14]
    RLjdqd3_33 = ROjdqd3_92*s.dpt[3,14]
    ORjdqd3_23 = -OMjdqd3_12*RLjdqd3_33
    ORjdqd3_33 = OMjdqd3_12*RLjdqd3_23
    Apqpjdqd3_23 = Apqpjdqd3_22-OMjdqd3_12*ORjdqd3_33
    Apqpjdqd3_33 = Apqpjdqd3_32+OMjdqd3_12*ORjdqd3_23
    RLjdqd4_22 = s.dpt[2,11]*C10
    RLjdqd4_32 = s.dpt[2,11]*S10
    ORjdqd4_22 = -RLjdqd4_32*qd[10]
    ORjdqd4_32 = RLjdqd4_22*qd[10]
    Apqpjdqd4_22 = -ORjdqd4_32*qd[10]
    Apqpjdqd4_32 = ORjdqd4_22*qd[10]
    jdqd5 = Apqpjdqd3_23-Apqpjdqd4_22
    jdqd6 = Apqpjdqd3_33-Apqpjdqd4_32
    Jdqd[1] = jdqd2
    Jdqd[2] = jdqd3
    Jdqd[3] = jdqd5
    Jdqd[4] = jdqd6

# Number of continuation lines = 0


