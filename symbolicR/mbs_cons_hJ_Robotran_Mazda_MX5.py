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
    S13 = sin(q[13])
    C13 = cos(q[13])
    S17 = sin(q[17])
    C17 = cos(q[17])
    S18 = sin(q[18])
    C18 = cos(q[18])
    S14 = sin(q[14])
    C14 = cos(q[14])
    S15 = sin(q[15])
    C15 = cos(q[15])
    S16 = sin(q[16])
    C16 = cos(q[16])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

    RLlp1_22 = s.dpt[2,8]*C7
    RLlp1_32 = s.dpt[2,8]*S7
    POlp1_22 = RLlp1_22+s.dpt[2,1]
    POlp1_32 = RLlp1_32+s.dpt[3,1]
    ROlp2_82 = -C8*S9-S8*C9
    ROlp2_92 = C8*C9-S8*S9
    RLlp2_22 = s.dpt[2,11]*C8
    RLlp2_32 = s.dpt[2,11]*S8
    POlp2_22 = RLlp2_22+s.dpt[2,2]
    POlp2_32 = RLlp2_32+s.dpt[3,2]
    RLlp2_23 = ROlp2_82*s.dpt[3,12]
    RLlp2_33 = ROlp2_92*s.dpt[3,12]
    POlp2_23 = POlp2_22+RLlp2_23
    POlp2_33 = POlp2_32+RLlp2_33
    JTlp2_23_1 = -RLlp2_32-RLlp2_33
    JTlp2_33_1 = RLlp2_22+RLlp2_23
    h_2 = POlp1_22-POlp2_23
    h_3 = POlp1_32-POlp2_33
    ROlp3_82 = -C11*S12-S11*C12
    ROlp3_92 = C11*C12-S11*S12
    RLlp3_22 = s.dpt[2,16]*C11
    RLlp3_32 = s.dpt[2,16]*S11
    POlp3_22 = RLlp3_22+s.dpt[2,5]
    POlp3_32 = RLlp3_32+s.dpt[3,5]
    RLlp3_23 = ROlp3_82*s.dpt[3,17]
    RLlp3_33 = ROlp3_92*s.dpt[3,17]
    POlp3_23 = POlp3_22+RLlp3_23
    POlp3_33 = POlp3_32+RLlp3_33
    JTlp3_23_1 = -RLlp3_32-RLlp3_33
    JTlp3_33_1 = RLlp3_22+RLlp3_23
    RLlp4_22 = s.dpt[2,13]*C10
    RLlp4_32 = s.dpt[2,13]*S10
    POlp4_22 = RLlp4_22+s.dpt[2,4]
    POlp4_32 = RLlp4_32+s.dpt[3,4]
    h_5 = POlp3_23-POlp4_22
    h_6 = POlp3_33-POlp4_32
    ROlp5_42 = S13*S17
    ROlp5_62 = C13*S17
    ROlp5_72 = S13*C17
    ROlp5_92 = C13*C17
    ROlp5_73 = ROlp5_72*C18+C13*S18
    ROlp5_83 = -S17*C18
    ROlp5_93 = ROlp5_92*C18-S13*S18
    RLlp5_14 = ROlp5_73*s.dpt[3,21]
    RLlp5_24 = ROlp5_83*s.dpt[3,21]
    RLlp5_34 = ROlp5_93*s.dpt[3,21]
    POlp5_14 = RLlp5_14+s.dpt[1,7]
    POlp5_24 = RLlp5_24+s.dpt[2,18]
    POlp5_34 = RLlp5_34+s.dpt[3,7]
    JTlp5_14_2 = RLlp5_24*S13
    JTlp5_24_2 = -RLlp5_14*S13-RLlp5_34*C13
    JTlp5_34_2 = RLlp5_24*C13
    JTlp5_14_3 = -RLlp5_24*ROlp5_62+RLlp5_34*C17
    JTlp5_24_3 = RLlp5_14*ROlp5_62-RLlp5_34*ROlp5_42
    JTlp5_34_3 = -RLlp5_14*C17+RLlp5_24*ROlp5_42
    RLlp6_22 = s.dpt[2,15]*C10
    RLlp6_32 = s.dpt[2,15]*S10
    POlp6_12 = s.dpt[1,15]+s.dpt[1,4]
    POlp6_22 = RLlp6_22+s.dpt[2,4]
    POlp6_32 = RLlp6_32+s.dpt[3,4]
    h_7 = POlp5_14-POlp6_12
    h_8 = POlp5_24-POlp6_22
    h_9 = POlp5_34-POlp6_32
    ROlp7_12 = C13*C14-S13*S14
    ROlp7_32 = -C13*S14-S13*C14
    ROlp7_72 = C13*S14+S13*C14
    ROlp7_92 = C13*C14-S13*S14
    ROlp7_43 = ROlp7_72*S15
    ROlp7_63 = ROlp7_92*S15
    ROlp7_73 = ROlp7_72*C15
    ROlp7_93 = ROlp7_92*C15
    ROlp7_74 = ROlp7_12*S16+ROlp7_73*C16
    ROlp7_84 = -S15*C16
    ROlp7_94 = ROlp7_32*S16+ROlp7_93*C16
    RLlp7_15 = ROlp7_74*s.dpt[3,20]
    RLlp7_25 = ROlp7_84*s.dpt[3,20]
    RLlp7_35 = ROlp7_94*s.dpt[3,20]
    POlp7_15 = RLlp7_15+s.dpt[1,7]
    POlp7_25 = RLlp7_25+s.dpt[2,19]
    POlp7_35 = RLlp7_35+s.dpt[3,7]
    JTlp7_15_3 = -RLlp7_25*ROlp7_32
    JTlp7_25_3 = RLlp7_15*ROlp7_32-RLlp7_35*ROlp7_12
    JTlp7_35_3 = RLlp7_25*ROlp7_12
    JTlp7_15_4 = -RLlp7_25*ROlp7_63+RLlp7_35*C15
    JTlp7_25_4 = RLlp7_15*ROlp7_63-RLlp7_35*ROlp7_43
    JTlp7_35_4 = -RLlp7_15*C15+RLlp7_25*ROlp7_43
    RLlp8_22 = s.dpt[2,10]*C7
    RLlp8_32 = s.dpt[2,10]*S7
    POlp8_12 = s.dpt[1,10]+s.dpt[1,1]
    POlp8_22 = RLlp8_22+s.dpt[2,1]
    POlp8_32 = RLlp8_32+s.dpt[3,1]
    h_10 = POlp7_15-POlp8_12
    h_11 = POlp7_25-POlp8_22
    h_12 = POlp7_35-POlp8_32
    h[1] = h_2
    h[2] = h_3
    h[3] = h_5
    h[4] = h_6
    h[5] = h_7
    h[6] = h_8
    h[7] = h_9
    h[8] = h_10
    h[9] = h_11
    h[10] = h_12
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
    Jac[1,13] = 0
    Jac[1,14] = 0
    Jac[1,15] = 0
    Jac[1,16] = 0
    Jac[1,17] = 0
    Jac[1,18] = 0
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
    Jac[2,13] = 0
    Jac[2,14] = 0
    Jac[2,15] = 0
    Jac[2,16] = 0
    Jac[2,17] = 0
    Jac[2,18] = 0
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
    Jac[3,13] = 0
    Jac[3,14] = 0
    Jac[3,15] = 0
    Jac[3,16] = 0
    Jac[3,17] = 0
    Jac[3,18] = 0
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
    Jac[4,13] = 0
    Jac[4,14] = 0
    Jac[4,15] = 0
    Jac[4,16] = 0
    Jac[4,17] = 0
    Jac[4,18] = 0
    Jac[5,1] = 0
    Jac[5,2] = 0
    Jac[5,3] = 0
    Jac[5,4] = 0
    Jac[5,5] = 0
    Jac[5,6] = 0
    Jac[5,7] = 0
    Jac[5,8] = 0
    Jac[5,9] = 0
    Jac[5,10] = 0
    Jac[5,11] = 0
    Jac[5,12] = 0
    Jac[5,13] = RLlp5_34
    Jac[5,14] = 0
    Jac[5,15] = 0
    Jac[5,16] = 0
    Jac[5,17] = JTlp5_14_2
    Jac[5,18] = JTlp5_14_3
    Jac[6,1] = 0
    Jac[6,2] = 0
    Jac[6,3] = 0
    Jac[6,4] = 0
    Jac[6,5] = 0
    Jac[6,6] = 0
    Jac[6,7] = 0
    Jac[6,8] = 0
    Jac[6,9] = 0
    Jac[6,10] = RLlp6_32
    Jac[6,11] = 0
    Jac[6,12] = 0
    Jac[6,13] = 0
    Jac[6,14] = 0
    Jac[6,15] = 0
    Jac[6,16] = 0
    Jac[6,17] = JTlp5_24_2
    Jac[6,18] = JTlp5_24_3
    Jac[7,1] = 0
    Jac[7,2] = 0
    Jac[7,3] = 0
    Jac[7,4] = 0
    Jac[7,5] = 0
    Jac[7,6] = 0
    Jac[7,7] = 0
    Jac[7,8] = 0
    Jac[7,9] = 0
    Jac[7,10] = -RLlp6_22
    Jac[7,11] = 0
    Jac[7,12] = 0
    Jac[7,13] = -RLlp5_14
    Jac[7,14] = 0
    Jac[7,15] = 0
    Jac[7,16] = 0
    Jac[7,17] = JTlp5_34_2
    Jac[7,18] = JTlp5_34_3
    Jac[8,1] = 0
    Jac[8,2] = 0
    Jac[8,3] = 0
    Jac[8,4] = 0
    Jac[8,5] = 0
    Jac[8,6] = 0
    Jac[8,7] = 0
    Jac[8,8] = 0
    Jac[8,9] = 0
    Jac[8,10] = 0
    Jac[8,11] = 0
    Jac[8,12] = 0
    Jac[8,13] = RLlp7_35
    Jac[8,14] = RLlp7_35
    Jac[8,15] = JTlp7_15_3
    Jac[8,16] = JTlp7_15_4
    Jac[8,17] = 0
    Jac[8,18] = 0
    Jac[9,1] = 0
    Jac[9,2] = 0
    Jac[9,3] = 0
    Jac[9,4] = 0
    Jac[9,5] = 0
    Jac[9,6] = 0
    Jac[9,7] = RLlp8_32
    Jac[9,8] = 0
    Jac[9,9] = 0
    Jac[9,10] = 0
    Jac[9,11] = 0
    Jac[9,12] = 0
    Jac[9,13] = 0
    Jac[9,14] = 0
    Jac[9,15] = JTlp7_25_3
    Jac[9,16] = JTlp7_25_4
    Jac[9,17] = 0
    Jac[9,18] = 0
    Jac[10,1] = 0
    Jac[10,2] = 0
    Jac[10,3] = 0
    Jac[10,4] = 0
    Jac[10,5] = 0
    Jac[10,6] = 0
    Jac[10,7] = -RLlp8_22
    Jac[10,8] = 0
    Jac[10,9] = 0
    Jac[10,10] = 0
    Jac[10,11] = 0
    Jac[10,12] = 0
    Jac[10,13] = -RLlp7_15
    Jac[10,14] = -RLlp7_15
    Jac[10,15] = JTlp7_35_3
    Jac[10,16] = JTlp7_35_4
    Jac[10,17] = 0
    Jac[10,18] = 0

# Number of continuation lines = 0


