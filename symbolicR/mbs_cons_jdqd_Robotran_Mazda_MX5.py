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
#	==> Generation Date: Tue Mar 31 18:21:50 2026
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Robotran_Mazda_MX5
#
#	==> Number of joints: 43
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
    S12 = sin(q[12])
    C12 = cos(q[12])
    S13 = sin(q[13])
    C13 = cos(q[13])
    S11 = sin(q[11])
    C11 = cos(q[11])
    S15 = sin(q[15])
    C15 = cos(q[15])
    S19 = sin(q[19])
    C19 = cos(q[19])
    S20 = sin(q[20])
    C20 = cos(q[20])
    S16 = sin(q[16])
    C16 = cos(q[16])
    S17 = sin(q[17])
    C17 = cos(q[17])
    S18 = sin(q[18])
    C18 = cos(q[18])
    S21 = sin(q[21])
    C21 = cos(q[21])
    S22 = sin(q[22])
    C22 = cos(q[22])
    S23 = sin(q[23])
    C23 = cos(q[23])
    S24 = sin(q[24])
    C24 = cos(q[24])
    S26 = sin(q[26])
    C26 = cos(q[26])
    S32 = sin(q[32])
    C32 = cos(q[32])
    S27 = sin(q[27])
    C27 = cos(q[27])
    S28 = sin(q[28])
    C28 = cos(q[28])
    S29 = sin(q[29])
    C29 = cos(q[29])
    S30 = sin(q[30])
    C30 = cos(q[30])
    S33 = sin(q[33])
    C33 = cos(q[33])
    S37 = sin(q[37])
    C37 = cos(q[37])
    S38 = sin(q[38])
    C38 = cos(q[38])
    S34 = sin(q[34])
    C34 = cos(q[34])
    S35 = sin(q[35])
    C35 = cos(q[35])
    S36 = sin(q[36])
    C36 = cos(q[36])
    S42 = sin(q[42])
    C42 = cos(q[42])
    S43 = sin(q[43])
    C43 = cos(q[43])
    S40 = sin(q[40])
    C40 = cos(q[40])
    S41 = sin(q[41])
    C41 = cos(q[41])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    RLjdqd1_22 = s.dpt[2,16]*C7
    RLjdqd1_32 = s.dpt[2,16]*S7
    ORjdqd1_22 = -RLjdqd1_32*qd[7]
    ORjdqd1_32 = RLjdqd1_22*qd[7]
    Apqpjdqd1_22 = -ORjdqd1_32*qd[7]
    Apqpjdqd1_32 = ORjdqd1_22*qd[7]
    ROjdqd2_82 = -C8*S9-S8*C9
    ROjdqd2_92 = C8*C9-S8*S9
    RLjdqd2_22 = s.dpt[2,19]*C8
    RLjdqd2_32 = s.dpt[2,19]*S8
    OMjdqd2_12 = qd[8]+qd[9]
    ORjdqd2_22 = -RLjdqd2_32*qd[8]
    ORjdqd2_32 = RLjdqd2_22*qd[8]
    Apqpjdqd2_22 = -ORjdqd2_32*qd[8]
    Apqpjdqd2_32 = ORjdqd2_22*qd[8]
    RLjdqd2_23 = ROjdqd2_82*s.dpt[3,20]
    RLjdqd2_33 = ROjdqd2_92*s.dpt[3,20]
    ORjdqd2_23 = -OMjdqd2_12*RLjdqd2_33
    ORjdqd2_33 = OMjdqd2_12*RLjdqd2_23
    Apqpjdqd2_23 = Apqpjdqd2_22-OMjdqd2_12*ORjdqd2_33
    Apqpjdqd2_33 = Apqpjdqd2_32+OMjdqd2_12*ORjdqd2_23
    jdqd2 = Apqpjdqd1_22-Apqpjdqd2_23
    jdqd3 = Apqpjdqd1_32-Apqpjdqd2_33
    ROjdqd3_82 = -C12*S13-S12*C13
    ROjdqd3_92 = C12*C13-S12*S13
    RLjdqd3_22 = s.dpt[2,26]*C12
    RLjdqd3_32 = s.dpt[2,26]*S12
    OMjdqd3_12 = qd[12]+qd[13]
    ORjdqd3_22 = -RLjdqd3_32*qd[12]
    ORjdqd3_32 = RLjdqd3_22*qd[12]
    Apqpjdqd3_22 = -ORjdqd3_32*qd[12]
    Apqpjdqd3_32 = ORjdqd3_22*qd[12]
    RLjdqd3_23 = ROjdqd3_82*s.dpt[3,27]
    RLjdqd3_33 = ROjdqd3_92*s.dpt[3,27]
    ORjdqd3_23 = -OMjdqd3_12*RLjdqd3_33
    ORjdqd3_33 = OMjdqd3_12*RLjdqd3_23
    Apqpjdqd3_23 = Apqpjdqd3_22-OMjdqd3_12*ORjdqd3_33
    Apqpjdqd3_33 = Apqpjdqd3_32+OMjdqd3_12*ORjdqd3_23
    RLjdqd4_22 = s.dpt[2,23]*C11
    RLjdqd4_32 = s.dpt[2,23]*S11
    ORjdqd4_22 = -RLjdqd4_32*qd[11]
    ORjdqd4_32 = RLjdqd4_22*qd[11]
    Apqpjdqd4_22 = -ORjdqd4_32*qd[11]
    Apqpjdqd4_32 = ORjdqd4_22*qd[11]
    jdqd5 = Apqpjdqd3_23-Apqpjdqd4_22
    jdqd6 = Apqpjdqd3_33-Apqpjdqd4_32
    ROjdqd5_42 = S15*S19
    ROjdqd5_62 = C15*S19
    ROjdqd5_72 = S15*C19
    ROjdqd5_92 = C15*C19
    ROjdqd5_73 = ROjdqd5_72*C20+C15*S20
    ROjdqd5_83 = -S19*C20
    ROjdqd5_93 = ROjdqd5_92*C20-S15*S20
    RLjdqd5_12 = s.dpt[1,30]*C15
    RLjdqd5_32 = -s.dpt[1,30]*S15
    OMjdqd5_12 = qd[19]*C15
    OMjdqd5_32 = -qd[19]*S15
    ORjdqd5_12 = RLjdqd5_32*qd[15]
    ORjdqd5_32 = -RLjdqd5_12*qd[15]
    Ompqpjdqd5_12 = -qd[15]*qd[19]*S15
    Ompqpjdqd5_32 = -qd[15]*qd[19]*C15
    Apqpjdqd5_12 = ORjdqd5_32*qd[15]
    Apqpjdqd5_32 = -ORjdqd5_12*qd[15]
    OMjdqd5_13 = OMjdqd5_12+ROjdqd5_42*qd[20]
    OMjdqd5_23 = qd[15]+qd[20]*C19
    OMjdqd5_33 = OMjdqd5_32+ROjdqd5_62*qd[20]
    Ompqpjdqd5_13 = Ompqpjdqd5_12+qd[20]*(-OMjdqd5_32*C19+ROjdqd5_62*qd[15])
    Ompqpjdqd5_23 = qd[20]*(-OMjdqd5_12*ROjdqd5_62+OMjdqd5_32*ROjdqd5_42)
    Ompqpjdqd5_33 = Ompqpjdqd5_32+qd[20]*(OMjdqd5_12*C19-ROjdqd5_42*qd[15])
    RLjdqd5_14 = ROjdqd5_73*s.dpt[3,33]
    RLjdqd5_24 = ROjdqd5_83*s.dpt[3,33]
    RLjdqd5_34 = ROjdqd5_93*s.dpt[3,33]
    ORjdqd5_14 = OMjdqd5_23*RLjdqd5_34-OMjdqd5_33*RLjdqd5_24
    ORjdqd5_24 = -OMjdqd5_13*RLjdqd5_34+OMjdqd5_33*RLjdqd5_14
    ORjdqd5_34 = OMjdqd5_13*RLjdqd5_24-OMjdqd5_23*RLjdqd5_14
    Apqpjdqd5_14 = Apqpjdqd5_12+OMjdqd5_23*ORjdqd5_34-OMjdqd5_33*ORjdqd5_24+Ompqpjdqd5_23*RLjdqd5_34-Ompqpjdqd5_33* \
 	  RLjdqd5_24
    Apqpjdqd5_24 = -OMjdqd5_13*ORjdqd5_34+OMjdqd5_33*ORjdqd5_14-Ompqpjdqd5_13*RLjdqd5_34+Ompqpjdqd5_33*RLjdqd5_14
    Apqpjdqd5_34 = Apqpjdqd5_32+OMjdqd5_13*ORjdqd5_24-OMjdqd5_23*ORjdqd5_14+Ompqpjdqd5_13*RLjdqd5_24-Ompqpjdqd5_23* \
 	  RLjdqd5_14
    RLjdqd6_22 = s.dpt[2,25]*C11
    RLjdqd6_32 = s.dpt[2,25]*S11
    ORjdqd6_22 = -RLjdqd6_32*qd[11]
    ORjdqd6_32 = RLjdqd6_22*qd[11]
    Apqpjdqd6_22 = -ORjdqd6_32*qd[11]
    Apqpjdqd6_32 = ORjdqd6_22*qd[11]
    jdqd8 = Apqpjdqd5_24-Apqpjdqd6_22
    jdqd9 = Apqpjdqd5_34-Apqpjdqd6_32
    ROjdqd7_12 = C15*C16-S15*S16
    ROjdqd7_32 = -C15*S16-S15*C16
    ROjdqd7_72 = C15*S16+S15*C16
    ROjdqd7_92 = C15*C16-S15*S16
    ROjdqd7_43 = ROjdqd7_72*S17
    ROjdqd7_63 = ROjdqd7_92*S17
    ROjdqd7_73 = ROjdqd7_72*C17
    ROjdqd7_93 = ROjdqd7_92*C17
    ROjdqd7_74 = ROjdqd7_12*S18+ROjdqd7_73*C18
    ROjdqd7_84 = -S17*C18
    ROjdqd7_94 = ROjdqd7_32*S18+ROjdqd7_93*C18
    OMjdqd7_22 = qd[15]+qd[16]
    RLjdqd7_13 = ROjdqd7_12*s.dpt[1,31]
    RLjdqd7_33 = ROjdqd7_32*s.dpt[1,31]
    OMjdqd7_13 = ROjdqd7_12*qd[17]
    OMjdqd7_33 = ROjdqd7_32*qd[17]
    ORjdqd7_13 = OMjdqd7_22*RLjdqd7_33
    ORjdqd7_33 = -OMjdqd7_22*RLjdqd7_13
    Ompqpjdqd7_13 = OMjdqd7_22*ROjdqd7_32*qd[17]
    Ompqpjdqd7_33 = -OMjdqd7_22*ROjdqd7_12*qd[17]
    Apqpjdqd7_13 = OMjdqd7_22*ORjdqd7_33
    Apqpjdqd7_33 = -OMjdqd7_22*ORjdqd7_13
    OMjdqd7_14 = OMjdqd7_13+ROjdqd7_43*qd[18]
    OMjdqd7_24 = OMjdqd7_22+qd[18]*C17
    OMjdqd7_34 = OMjdqd7_33+ROjdqd7_63*qd[18]
    Ompqpjdqd7_14 = Ompqpjdqd7_13+qd[18]*(OMjdqd7_22*ROjdqd7_63-OMjdqd7_33*C17)
    Ompqpjdqd7_24 = qd[18]*(-OMjdqd7_13*ROjdqd7_63+OMjdqd7_33*ROjdqd7_43)
    Ompqpjdqd7_34 = Ompqpjdqd7_33+qd[18]*(OMjdqd7_13*C17-OMjdqd7_22*ROjdqd7_43)
    RLjdqd7_15 = ROjdqd7_74*s.dpt[3,32]
    RLjdqd7_25 = ROjdqd7_84*s.dpt[3,32]
    RLjdqd7_35 = ROjdqd7_94*s.dpt[3,32]
    ORjdqd7_15 = OMjdqd7_24*RLjdqd7_35-OMjdqd7_34*RLjdqd7_25
    ORjdqd7_25 = -OMjdqd7_14*RLjdqd7_35+OMjdqd7_34*RLjdqd7_15
    ORjdqd7_35 = OMjdqd7_14*RLjdqd7_25-OMjdqd7_24*RLjdqd7_15
    Apqpjdqd7_15 = Apqpjdqd7_13+OMjdqd7_24*ORjdqd7_35-OMjdqd7_34*ORjdqd7_25+Ompqpjdqd7_24*RLjdqd7_35-Ompqpjdqd7_34* \
 	  RLjdqd7_25
    Apqpjdqd7_25 = -OMjdqd7_14*ORjdqd7_35+OMjdqd7_34*ORjdqd7_15-Ompqpjdqd7_14*RLjdqd7_35+Ompqpjdqd7_34*RLjdqd7_15
    Apqpjdqd7_35 = Apqpjdqd7_33+OMjdqd7_14*ORjdqd7_25-OMjdqd7_24*ORjdqd7_15+Ompqpjdqd7_14*RLjdqd7_25-Ompqpjdqd7_24* \
 	  RLjdqd7_15
    RLjdqd8_22 = s.dpt[2,18]*C7
    RLjdqd8_32 = s.dpt[2,18]*S7
    ORjdqd8_22 = -RLjdqd8_32*qd[7]
    ORjdqd8_32 = RLjdqd8_22*qd[7]
    Apqpjdqd8_22 = -ORjdqd8_32*qd[7]
    Apqpjdqd8_32 = ORjdqd8_22*qd[7]
    jdqd11 = Apqpjdqd7_25-Apqpjdqd8_22
    jdqd12 = Apqpjdqd7_35-Apqpjdqd8_32
    ROjdqd9_52 = C21*C22-S21*S22
    ROjdqd9_62 = C21*S22+S21*C22
    ROjdqd9_82 = -C21*S22-S21*C22
    ROjdqd9_92 = C21*C22-S21*S22
    ROjdqd9_83 = ROjdqd9_82*C23
    ROjdqd9_93 = ROjdqd9_92*C23
    RLjdqd9_22 = s.dpt[2,34]*C21
    RLjdqd9_32 = s.dpt[2,34]*S21
    OMjdqd9_12 = qd[21]+qd[22]
    ORjdqd9_22 = -RLjdqd9_32*qd[21]
    ORjdqd9_32 = RLjdqd9_22*qd[21]
    Apqpjdqd9_22 = -ORjdqd9_32*qd[21]
    Apqpjdqd9_32 = ORjdqd9_22*qd[21]
    OMjdqd9_23 = ROjdqd9_52*qd[23]
    OMjdqd9_33 = ROjdqd9_62*qd[23]
    Ompqpjdqd9_23 = -OMjdqd9_12*ROjdqd9_62*qd[23]
    Ompqpjdqd9_33 = OMjdqd9_12*ROjdqd9_52*qd[23]
    OMjdqd9_14 = OMjdqd9_12+qd[24]*S23
    OMjdqd9_24 = OMjdqd9_23+ROjdqd9_83*qd[24]
    OMjdqd9_34 = OMjdqd9_33+ROjdqd9_93*qd[24]
    Ompqpjdqd9_14 = qd[24]*(OMjdqd9_23*ROjdqd9_93-OMjdqd9_33*ROjdqd9_83)
    Ompqpjdqd9_24 = Ompqpjdqd9_23+qd[24]*(-OMjdqd9_12*ROjdqd9_93+OMjdqd9_33*S23)
    Ompqpjdqd9_34 = Ompqpjdqd9_33+qd[24]*(OMjdqd9_12*ROjdqd9_83-OMjdqd9_23*S23)
    RLjdqd9_15 = s.dpt[3,35]*S23
    RLjdqd9_25 = ROjdqd9_83*s.dpt[3,35]
    RLjdqd9_35 = ROjdqd9_93*s.dpt[3,35]
    ORjdqd9_15 = OMjdqd9_24*RLjdqd9_35-OMjdqd9_34*RLjdqd9_25
    ORjdqd9_25 = -OMjdqd9_14*RLjdqd9_35+OMjdqd9_34*RLjdqd9_15
    ORjdqd9_35 = OMjdqd9_14*RLjdqd9_25-OMjdqd9_24*RLjdqd9_15
    Apqpjdqd9_15 = OMjdqd9_24*ORjdqd9_35-OMjdqd9_34*ORjdqd9_25+Ompqpjdqd9_24*RLjdqd9_35-Ompqpjdqd9_34*RLjdqd9_25
    Apqpjdqd9_25 = Apqpjdqd9_22-OMjdqd9_14*ORjdqd9_35+OMjdqd9_34*ORjdqd9_15-Ompqpjdqd9_14*RLjdqd9_35+Ompqpjdqd9_34* \
 	  RLjdqd9_15
    Apqpjdqd9_35 = Apqpjdqd9_32+OMjdqd9_14*ORjdqd9_25-OMjdqd9_24*ORjdqd9_15+Ompqpjdqd9_14*RLjdqd9_25-Ompqpjdqd9_24* \
 	  RLjdqd9_15
    RLjdqd10_22 = s.dpt[2,39]*C26
    RLjdqd10_32 = s.dpt[2,39]*S26
    ORjdqd10_22 = -RLjdqd10_32*qd[26]
    ORjdqd10_32 = RLjdqd10_22*qd[26]
    Apqpjdqd10_22 = -ORjdqd10_32*qd[26]
    Apqpjdqd10_32 = ORjdqd10_22*qd[26]
    jdqd14 = -Apqpjdqd10_22+Apqpjdqd9_25
    jdqd15 = -Apqpjdqd10_32+Apqpjdqd9_35
    RLjdqd11_22 = s.dpt[2,47]*C32
    RLjdqd11_32 = s.dpt[2,47]*S32
    ORjdqd11_22 = -RLjdqd11_32*qd[32]
    ORjdqd11_32 = RLjdqd11_22*qd[32]
    Apqpjdqd11_22 = -ORjdqd11_32*qd[32]
    Apqpjdqd11_32 = ORjdqd11_22*qd[32]
    ROjdqd12_52 = C27*C28-S27*S28
    ROjdqd12_62 = C27*S28+S27*C28
    ROjdqd12_82 = -C27*S28-S27*C28
    ROjdqd12_92 = C27*C28-S27*S28
    ROjdqd12_83 = ROjdqd12_82*C29
    ROjdqd12_93 = ROjdqd12_92*C29
    RLjdqd12_22 = s.dpt[2,42]*C27
    RLjdqd12_32 = s.dpt[2,42]*S27
    OMjdqd12_12 = qd[27]+qd[28]
    ORjdqd12_22 = -RLjdqd12_32*qd[27]
    ORjdqd12_32 = RLjdqd12_22*qd[27]
    Apqpjdqd12_22 = -ORjdqd12_32*qd[27]
    Apqpjdqd12_32 = ORjdqd12_22*qd[27]
    OMjdqd12_23 = ROjdqd12_52*qd[29]
    OMjdqd12_33 = ROjdqd12_62*qd[29]
    Ompqpjdqd12_23 = -OMjdqd12_12*ROjdqd12_62*qd[29]
    Ompqpjdqd12_33 = OMjdqd12_12*ROjdqd12_52*qd[29]
    OMjdqd12_14 = OMjdqd12_12+qd[30]*S29
    OMjdqd12_24 = OMjdqd12_23+ROjdqd12_83*qd[30]
    OMjdqd12_34 = OMjdqd12_33+ROjdqd12_93*qd[30]
    Ompqpjdqd12_14 = qd[30]*(OMjdqd12_23*ROjdqd12_93-OMjdqd12_33*ROjdqd12_83)
    Ompqpjdqd12_24 = Ompqpjdqd12_23+qd[30]*(-OMjdqd12_12*ROjdqd12_93+OMjdqd12_33*S29)
    Ompqpjdqd12_34 = Ompqpjdqd12_33+qd[30]*(OMjdqd12_12*ROjdqd12_83-OMjdqd12_23*S29)
    RLjdqd12_15 = s.dpt[3,43]*S29
    RLjdqd12_25 = ROjdqd12_83*s.dpt[3,43]
    RLjdqd12_35 = ROjdqd12_93*s.dpt[3,43]
    ORjdqd12_15 = OMjdqd12_24*RLjdqd12_35-OMjdqd12_34*RLjdqd12_25
    ORjdqd12_25 = -OMjdqd12_14*RLjdqd12_35+OMjdqd12_34*RLjdqd12_15
    ORjdqd12_35 = OMjdqd12_14*RLjdqd12_25-OMjdqd12_24*RLjdqd12_15
    Apqpjdqd12_15 = OMjdqd12_24*ORjdqd12_35-OMjdqd12_34*ORjdqd12_25+Ompqpjdqd12_24*RLjdqd12_35-Ompqpjdqd12_34* \
 	  RLjdqd12_25
    Apqpjdqd12_25 = Apqpjdqd12_22-OMjdqd12_14*ORjdqd12_35+OMjdqd12_34*ORjdqd12_15-Ompqpjdqd12_14*RLjdqd12_35+ \
 	  Ompqpjdqd12_34*RLjdqd12_15
    Apqpjdqd12_35 = Apqpjdqd12_32+OMjdqd12_14*ORjdqd12_25-OMjdqd12_24*ORjdqd12_15+Ompqpjdqd12_14*RLjdqd12_25- \
 	  Ompqpjdqd12_24*RLjdqd12_15
    jdqd17 = Apqpjdqd11_22-Apqpjdqd12_25
    jdqd18 = Apqpjdqd11_32-Apqpjdqd12_35
    ROjdqd13_42 = S33*S37
    ROjdqd13_62 = C33*S37
    ROjdqd13_72 = S33*C37
    ROjdqd13_92 = C33*C37
    ROjdqd13_73 = ROjdqd13_72*C38+C33*S38
    ROjdqd13_83 = -S37*C38
    ROjdqd13_93 = ROjdqd13_92*C38-S33*S38
    RLjdqd13_12 = s.dpt[1,50]*C33
    RLjdqd13_32 = -s.dpt[1,50]*S33
    OMjdqd13_12 = qd[37]*C33
    OMjdqd13_32 = -qd[37]*S33
    ORjdqd13_12 = RLjdqd13_32*qd[33]
    ORjdqd13_32 = -RLjdqd13_12*qd[33]
    Ompqpjdqd13_12 = -qd[33]*qd[37]*S33
    Ompqpjdqd13_32 = -qd[33]*qd[37]*C33
    Apqpjdqd13_12 = ORjdqd13_32*qd[33]
    Apqpjdqd13_32 = -ORjdqd13_12*qd[33]
    OMjdqd13_13 = OMjdqd13_12+ROjdqd13_42*qd[38]
    OMjdqd13_23 = qd[33]+qd[38]*C37
    OMjdqd13_33 = OMjdqd13_32+ROjdqd13_62*qd[38]
    Ompqpjdqd13_13 = Ompqpjdqd13_12+qd[38]*(-OMjdqd13_32*C37+ROjdqd13_62*qd[33])
    Ompqpjdqd13_23 = qd[38]*(-OMjdqd13_12*ROjdqd13_62+OMjdqd13_32*ROjdqd13_42)
    Ompqpjdqd13_33 = Ompqpjdqd13_32+qd[38]*(OMjdqd13_12*C37-ROjdqd13_42*qd[33])
    RLjdqd13_14 = ROjdqd13_73*s.dpt[3,53]
    RLjdqd13_24 = ROjdqd13_83*s.dpt[3,53]
    RLjdqd13_34 = ROjdqd13_93*s.dpt[3,53]
    ORjdqd13_14 = OMjdqd13_23*RLjdqd13_34-OMjdqd13_33*RLjdqd13_24
    ORjdqd13_24 = -OMjdqd13_13*RLjdqd13_34+OMjdqd13_33*RLjdqd13_14
    ORjdqd13_34 = OMjdqd13_13*RLjdqd13_24-OMjdqd13_23*RLjdqd13_14
    Apqpjdqd13_14 = Apqpjdqd13_12+OMjdqd13_23*ORjdqd13_34-OMjdqd13_33*ORjdqd13_24+Ompqpjdqd13_23*RLjdqd13_34- \
 	  Ompqpjdqd13_33*RLjdqd13_24
    Apqpjdqd13_24 = -OMjdqd13_13*ORjdqd13_34+OMjdqd13_33*ORjdqd13_14-Ompqpjdqd13_13*RLjdqd13_34+Ompqpjdqd13_33* \
 	  RLjdqd13_14
    Apqpjdqd13_34 = Apqpjdqd13_32+OMjdqd13_13*ORjdqd13_24-OMjdqd13_23*ORjdqd13_14+Ompqpjdqd13_13*RLjdqd13_24- \
 	  Ompqpjdqd13_23*RLjdqd13_14
    RLjdqd14_22 = s.dpt[2,41]*C26
    RLjdqd14_32 = s.dpt[2,41]*S26
    ORjdqd14_22 = -RLjdqd14_32*qd[26]
    ORjdqd14_32 = RLjdqd14_22*qd[26]
    Apqpjdqd14_22 = -ORjdqd14_32*qd[26]
    Apqpjdqd14_32 = ORjdqd14_22*qd[26]
    jdqd20 = Apqpjdqd13_24-Apqpjdqd14_22
    jdqd21 = Apqpjdqd13_34-Apqpjdqd14_32
    ROjdqd15_12 = C33*C34-S33*S34
    ROjdqd15_32 = -C33*S34-S33*C34
    ROjdqd15_72 = C33*S34+S33*C34
    ROjdqd15_92 = C33*C34-S33*S34
    ROjdqd15_43 = ROjdqd15_72*S35
    ROjdqd15_63 = ROjdqd15_92*S35
    ROjdqd15_73 = ROjdqd15_72*C35
    ROjdqd15_93 = ROjdqd15_92*C35
    ROjdqd15_74 = ROjdqd15_12*S36+ROjdqd15_73*C36
    ROjdqd15_84 = -S35*C36
    ROjdqd15_94 = ROjdqd15_32*S36+ROjdqd15_93*C36
    OMjdqd15_22 = qd[33]+qd[34]
    RLjdqd15_13 = ROjdqd15_12*s.dpt[1,51]
    RLjdqd15_33 = ROjdqd15_32*s.dpt[1,51]
    OMjdqd15_13 = ROjdqd15_12*qd[35]
    OMjdqd15_33 = ROjdqd15_32*qd[35]
    ORjdqd15_13 = OMjdqd15_22*RLjdqd15_33
    ORjdqd15_33 = -OMjdqd15_22*RLjdqd15_13
    Ompqpjdqd15_13 = OMjdqd15_22*ROjdqd15_32*qd[35]
    Ompqpjdqd15_33 = -OMjdqd15_22*ROjdqd15_12*qd[35]
    Apqpjdqd15_13 = OMjdqd15_22*ORjdqd15_33
    Apqpjdqd15_33 = -OMjdqd15_22*ORjdqd15_13
    OMjdqd15_14 = OMjdqd15_13+ROjdqd15_43*qd[36]
    OMjdqd15_24 = OMjdqd15_22+qd[36]*C35
    OMjdqd15_34 = OMjdqd15_33+ROjdqd15_63*qd[36]
    Ompqpjdqd15_14 = Ompqpjdqd15_13+qd[36]*(OMjdqd15_22*ROjdqd15_63-OMjdqd15_33*C35)
    Ompqpjdqd15_24 = qd[36]*(-OMjdqd15_13*ROjdqd15_63+OMjdqd15_33*ROjdqd15_43)
    Ompqpjdqd15_34 = Ompqpjdqd15_33+qd[36]*(OMjdqd15_13*C35-OMjdqd15_22*ROjdqd15_43)
    RLjdqd15_15 = ROjdqd15_74*s.dpt[3,52]
    RLjdqd15_25 = ROjdqd15_84*s.dpt[3,52]
    RLjdqd15_35 = ROjdqd15_94*s.dpt[3,52]
    ORjdqd15_15 = OMjdqd15_24*RLjdqd15_35-OMjdqd15_34*RLjdqd15_25
    ORjdqd15_25 = -OMjdqd15_14*RLjdqd15_35+OMjdqd15_34*RLjdqd15_15
    ORjdqd15_35 = OMjdqd15_14*RLjdqd15_25-OMjdqd15_24*RLjdqd15_15
    Apqpjdqd15_15 = Apqpjdqd15_13+OMjdqd15_24*ORjdqd15_35-OMjdqd15_34*ORjdqd15_25+Ompqpjdqd15_24*RLjdqd15_35- \
 	  Ompqpjdqd15_34*RLjdqd15_25
    Apqpjdqd15_25 = -OMjdqd15_14*ORjdqd15_35+OMjdqd15_34*ORjdqd15_15-Ompqpjdqd15_14*RLjdqd15_35+Ompqpjdqd15_34* \
 	  RLjdqd15_15
    Apqpjdqd15_35 = Apqpjdqd15_33+OMjdqd15_14*ORjdqd15_25-OMjdqd15_24*ORjdqd15_15+Ompqpjdqd15_14*RLjdqd15_25- \
 	  Ompqpjdqd15_24*RLjdqd15_15
    RLjdqd16_22 = s.dpt[2,49]*C32
    RLjdqd16_32 = s.dpt[2,49]*S32
    ORjdqd16_22 = -RLjdqd16_32*qd[32]
    ORjdqd16_32 = RLjdqd16_22*qd[32]
    Apqpjdqd16_22 = -ORjdqd16_32*qd[32]
    Apqpjdqd16_32 = ORjdqd16_22*qd[32]
    jdqd23 = Apqpjdqd15_25-Apqpjdqd16_22
    jdqd24 = Apqpjdqd15_35-Apqpjdqd16_32
    ROjdqd17_52 = C21*C22-S21*S22
    ROjdqd17_62 = C21*S22+S21*C22
    ROjdqd17_82 = -C21*S22-S21*C22
    ROjdqd17_92 = C21*C22-S21*S22
    ROjdqd17_23 = -ROjdqd17_82*S23
    ROjdqd17_33 = -ROjdqd17_92*S23
    ROjdqd17_83 = ROjdqd17_82*C23
    ROjdqd17_93 = ROjdqd17_92*C23
    ROjdqd17_14 = C23*C24
    ROjdqd17_24 = ROjdqd17_23*C24+ROjdqd17_52*S24
    ROjdqd17_34 = ROjdqd17_33*C24+ROjdqd17_62*S24
    ROjdqd17_44 = -C23*S24
    ROjdqd17_54 = -ROjdqd17_23*S24+ROjdqd17_52*C24
    ROjdqd17_64 = -ROjdqd17_33*S24+ROjdqd17_62*C24
    RLjdqd17_22 = s.dpt[2,34]*C21
    RLjdqd17_32 = s.dpt[2,34]*S21
    OMjdqd17_12 = qd[21]+qd[22]
    ORjdqd17_22 = -RLjdqd17_32*qd[21]
    ORjdqd17_32 = RLjdqd17_22*qd[21]
    Apqpjdqd17_22 = -ORjdqd17_32*qd[21]
    Apqpjdqd17_32 = ORjdqd17_22*qd[21]
    OMjdqd17_23 = ROjdqd17_52*qd[23]
    OMjdqd17_33 = ROjdqd17_62*qd[23]
    Ompqpjdqd17_23 = -OMjdqd17_12*ROjdqd17_62*qd[23]
    Ompqpjdqd17_33 = OMjdqd17_12*ROjdqd17_52*qd[23]
    OMjdqd17_14 = OMjdqd17_12+qd[24]*S23
    OMjdqd17_24 = OMjdqd17_23+ROjdqd17_83*qd[24]
    OMjdqd17_34 = OMjdqd17_33+ROjdqd17_93*qd[24]
    Ompqpjdqd17_14 = qd[24]*(OMjdqd17_23*ROjdqd17_93-OMjdqd17_33*ROjdqd17_83)
    Ompqpjdqd17_24 = Ompqpjdqd17_23+qd[24]*(-OMjdqd17_12*ROjdqd17_93+OMjdqd17_33*S23)
    Ompqpjdqd17_34 = Ompqpjdqd17_33+qd[24]*(OMjdqd17_12*ROjdqd17_83-OMjdqd17_23*S23)
    RLjdqd17_15 = ROjdqd17_14*s.dpt[1,37]+ROjdqd17_44*s.dpt[2,37]+s.dpt[3,37]*S23
    RLjdqd17_25 = ROjdqd17_24*s.dpt[1,37]+ROjdqd17_54*s.dpt[2,37]+ROjdqd17_83*s.dpt[3,37]
    RLjdqd17_35 = ROjdqd17_34*s.dpt[1,37]+ROjdqd17_64*s.dpt[2,37]+ROjdqd17_93*s.dpt[3,37]
    ORjdqd17_15 = OMjdqd17_24*RLjdqd17_35-OMjdqd17_34*RLjdqd17_25
    ORjdqd17_25 = -OMjdqd17_14*RLjdqd17_35+OMjdqd17_34*RLjdqd17_15
    ORjdqd17_35 = OMjdqd17_14*RLjdqd17_25-OMjdqd17_24*RLjdqd17_15
    Apqpjdqd17_15 = OMjdqd17_24*ORjdqd17_35-OMjdqd17_34*ORjdqd17_25+Ompqpjdqd17_24*RLjdqd17_35-Ompqpjdqd17_34* \
 	  RLjdqd17_25
    Apqpjdqd17_25 = Apqpjdqd17_22-OMjdqd17_14*ORjdqd17_35+OMjdqd17_34*ORjdqd17_15-Ompqpjdqd17_14*RLjdqd17_35+ \
 	  Ompqpjdqd17_34*RLjdqd17_15
    Apqpjdqd17_35 = Apqpjdqd17_32+OMjdqd17_14*ORjdqd17_25-OMjdqd17_24*ORjdqd17_15+Ompqpjdqd17_14*RLjdqd17_25- \
 	  Ompqpjdqd17_24*RLjdqd17_15
    ROjdqd18_53 = C42*C43
    ROjdqd18_63 = S42*C43
    OMjdqd18_23 = -qd[43]*S42
    OMjdqd18_33 = qd[43]*C42
    Ompqpjdqd18_23 = -qd[42]*qd[43]*C42
    Ompqpjdqd18_33 = -qd[42]*qd[43]*S42
    RLjdqd18_14 = -s.dpt[2,57]*S43
    RLjdqd18_24 = ROjdqd18_53*s.dpt[2,57]-s.dpt[3,57]*S42
    RLjdqd18_34 = ROjdqd18_63*s.dpt[2,57]+s.dpt[3,57]*C42
    ORjdqd18_14 = OMjdqd18_23*RLjdqd18_34-OMjdqd18_33*RLjdqd18_24
    ORjdqd18_24 = OMjdqd18_33*RLjdqd18_14-RLjdqd18_34*qd[42]
    ORjdqd18_34 = -OMjdqd18_23*RLjdqd18_14+RLjdqd18_24*qd[42]
    Apqpjdqd18_14 = OMjdqd18_23*ORjdqd18_34-OMjdqd18_33*ORjdqd18_24+Ompqpjdqd18_23*RLjdqd18_34-Ompqpjdqd18_33* \
 	  RLjdqd18_24
    Apqpjdqd18_24 = OMjdqd18_33*ORjdqd18_14-ORjdqd18_34*qd[42]+Ompqpjdqd18_33*RLjdqd18_14
    Apqpjdqd18_34 = -OMjdqd18_23*ORjdqd18_14+ORjdqd18_24*qd[42]-Ompqpjdqd18_23*RLjdqd18_14
    jdqd25 = Apqpjdqd17_15-Apqpjdqd18_14
    jdqd26 = Apqpjdqd17_25-Apqpjdqd18_24
    jdqd27 = Apqpjdqd17_35-Apqpjdqd18_34
    ROjdqd19_53 = C40*C41
    ROjdqd19_63 = S40*C41
    OMjdqd19_23 = -qd[41]*S40
    OMjdqd19_33 = qd[41]*C40
    Ompqpjdqd19_23 = -qd[40]*qd[41]*C40
    Ompqpjdqd19_33 = -qd[40]*qd[41]*S40
    RLjdqd19_14 = -s.dpt[2,56]*S41
    RLjdqd19_24 = ROjdqd19_53*s.dpt[2,56]-s.dpt[3,56]*S40
    RLjdqd19_34 = ROjdqd19_63*s.dpt[2,56]+s.dpt[3,56]*C40
    ORjdqd19_14 = OMjdqd19_23*RLjdqd19_34-OMjdqd19_33*RLjdqd19_24
    ORjdqd19_24 = OMjdqd19_33*RLjdqd19_14-RLjdqd19_34*qd[40]
    ORjdqd19_34 = -OMjdqd19_23*RLjdqd19_14+RLjdqd19_24*qd[40]
    Apqpjdqd19_14 = OMjdqd19_23*ORjdqd19_34-OMjdqd19_33*ORjdqd19_24+Ompqpjdqd19_23*RLjdqd19_34-Ompqpjdqd19_33* \
 	  RLjdqd19_24
    Apqpjdqd19_24 = OMjdqd19_33*ORjdqd19_14-ORjdqd19_34*qd[40]+Ompqpjdqd19_33*RLjdqd19_14
    Apqpjdqd19_34 = -OMjdqd19_23*ORjdqd19_14+ORjdqd19_24*qd[40]-Ompqpjdqd19_23*RLjdqd19_14
    ROjdqd20_52 = C27*C28-S27*S28
    ROjdqd20_62 = C27*S28+S27*C28
    ROjdqd20_82 = -C27*S28-S27*C28
    ROjdqd20_92 = C27*C28-S27*S28
    ROjdqd20_23 = -ROjdqd20_82*S29
    ROjdqd20_33 = -ROjdqd20_92*S29
    ROjdqd20_83 = ROjdqd20_82*C29
    ROjdqd20_93 = ROjdqd20_92*C29
    ROjdqd20_14 = C29*C30
    ROjdqd20_24 = ROjdqd20_23*C30+ROjdqd20_52*S30
    ROjdqd20_34 = ROjdqd20_33*C30+ROjdqd20_62*S30
    ROjdqd20_44 = -C29*S30
    ROjdqd20_54 = -ROjdqd20_23*S30+ROjdqd20_52*C30
    ROjdqd20_64 = -ROjdqd20_33*S30+ROjdqd20_62*C30
    RLjdqd20_22 = s.dpt[2,42]*C27
    RLjdqd20_32 = s.dpt[2,42]*S27
    OMjdqd20_12 = qd[27]+qd[28]
    ORjdqd20_22 = -RLjdqd20_32*qd[27]
    ORjdqd20_32 = RLjdqd20_22*qd[27]
    Apqpjdqd20_22 = -ORjdqd20_32*qd[27]
    Apqpjdqd20_32 = ORjdqd20_22*qd[27]
    OMjdqd20_23 = ROjdqd20_52*qd[29]
    OMjdqd20_33 = ROjdqd20_62*qd[29]
    Ompqpjdqd20_23 = -OMjdqd20_12*ROjdqd20_62*qd[29]
    Ompqpjdqd20_33 = OMjdqd20_12*ROjdqd20_52*qd[29]
    OMjdqd20_14 = OMjdqd20_12+qd[30]*S29
    OMjdqd20_24 = OMjdqd20_23+ROjdqd20_83*qd[30]
    OMjdqd20_34 = OMjdqd20_33+ROjdqd20_93*qd[30]
    Ompqpjdqd20_14 = qd[30]*(OMjdqd20_23*ROjdqd20_93-OMjdqd20_33*ROjdqd20_83)
    Ompqpjdqd20_24 = Ompqpjdqd20_23+qd[30]*(-OMjdqd20_12*ROjdqd20_93+OMjdqd20_33*S29)
    Ompqpjdqd20_34 = Ompqpjdqd20_33+qd[30]*(OMjdqd20_12*ROjdqd20_83-OMjdqd20_23*S29)
    RLjdqd20_15 = ROjdqd20_14*s.dpt[1,45]+ROjdqd20_44*s.dpt[2,45]+s.dpt[3,45]*S29
    RLjdqd20_25 = ROjdqd20_24*s.dpt[1,45]+ROjdqd20_54*s.dpt[2,45]+ROjdqd20_83*s.dpt[3,45]
    RLjdqd20_35 = ROjdqd20_34*s.dpt[1,45]+ROjdqd20_64*s.dpt[2,45]+ROjdqd20_93*s.dpt[3,45]
    ORjdqd20_15 = OMjdqd20_24*RLjdqd20_35-OMjdqd20_34*RLjdqd20_25
    ORjdqd20_25 = -OMjdqd20_14*RLjdqd20_35+OMjdqd20_34*RLjdqd20_15
    ORjdqd20_35 = OMjdqd20_14*RLjdqd20_25-OMjdqd20_24*RLjdqd20_15
    Apqpjdqd20_15 = OMjdqd20_24*ORjdqd20_35-OMjdqd20_34*ORjdqd20_25+Ompqpjdqd20_24*RLjdqd20_35-Ompqpjdqd20_34* \
 	  RLjdqd20_25
    Apqpjdqd20_25 = Apqpjdqd20_22-OMjdqd20_14*ORjdqd20_35+OMjdqd20_34*ORjdqd20_15-Ompqpjdqd20_14*RLjdqd20_35+ \
 	  Ompqpjdqd20_34*RLjdqd20_15
    Apqpjdqd20_35 = Apqpjdqd20_32+OMjdqd20_14*ORjdqd20_25-OMjdqd20_24*ORjdqd20_15+Ompqpjdqd20_14*RLjdqd20_25- \
 	  Ompqpjdqd20_24*RLjdqd20_15
    jdqd28 = Apqpjdqd19_14-Apqpjdqd20_15
    jdqd29 = Apqpjdqd19_24-Apqpjdqd20_25
    jdqd30 = Apqpjdqd19_34-Apqpjdqd20_35
    Jdqd[1] = jdqd2
    Jdqd[2] = jdqd3
    Jdqd[3] = jdqd5
    Jdqd[4] = jdqd6
    Jdqd[5] = Apqpjdqd5_14
    Jdqd[6] = jdqd8
    Jdqd[7] = jdqd9
    Jdqd[8] = Apqpjdqd7_15
    Jdqd[9] = jdqd11
    Jdqd[10] = jdqd12
    Jdqd[11] = Apqpjdqd9_15
    Jdqd[12] = jdqd14
    Jdqd[13] = jdqd15
    Jdqd[14] = -Apqpjdqd12_15
    Jdqd[15] = jdqd17
    Jdqd[16] = jdqd18
    Jdqd[17] = Apqpjdqd13_14
    Jdqd[18] = jdqd20
    Jdqd[19] = jdqd21
    Jdqd[20] = Apqpjdqd15_15
    Jdqd[21] = jdqd23
    Jdqd[22] = jdqd24
    Jdqd[23] = jdqd25
    Jdqd[24] = jdqd26
    Jdqd[25] = jdqd27
    Jdqd[26] = jdqd28
    Jdqd[27] = jdqd29
    Jdqd[28] = jdqd30

# Number of continuation lines = 1


