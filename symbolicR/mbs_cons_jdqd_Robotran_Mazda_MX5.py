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
#	==> Generation Date: Tue Apr 14 14:30:26 2026
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
    S7 = sin(q[7])
    C7 = cos(q[7])
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
    S32 = sin(q[32])
    C32 = cos(q[32])
    S42 = sin(q[42])
    C42 = cos(q[42])
    S43 = sin(q[43])
    C43 = cos(q[43])
    S27 = sin(q[27])
    C27 = cos(q[27])
    S28 = sin(q[28])
    C28 = cos(q[28])
    S29 = sin(q[29])
    C29 = cos(q[29])
    S30 = sin(q[30])
    C30 = cos(q[30])
    S8 = sin(q[8])
    C8 = cos(q[8])
    S9 = sin(q[9])
    C9 = cos(q[9])
    S40 = sin(q[40])
    C40 = cos(q[40])
    S41 = sin(q[41])
    C41 = cos(q[41])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    ROjdqd1_82 = -C12*S13-S12*C13
    ROjdqd1_92 = C12*C13-S12*S13
    RLjdqd1_22 = s.dpt[2,26]*C12
    RLjdqd1_32 = s.dpt[2,26]*S12
    OMjdqd1_12 = qd[12]+qd[13]
    ORjdqd1_22 = -RLjdqd1_32*qd[12]
    ORjdqd1_32 = RLjdqd1_22*qd[12]
    Apqpjdqd1_22 = -ORjdqd1_32*qd[12]
    Apqpjdqd1_32 = ORjdqd1_22*qd[12]
    RLjdqd1_23 = ROjdqd1_82*s.dpt[3,27]
    RLjdqd1_33 = ROjdqd1_92*s.dpt[3,27]
    ORjdqd1_23 = -OMjdqd1_12*RLjdqd1_33
    ORjdqd1_33 = OMjdqd1_12*RLjdqd1_23
    Apqpjdqd1_23 = Apqpjdqd1_22-OMjdqd1_12*ORjdqd1_33
    Apqpjdqd1_33 = Apqpjdqd1_32+OMjdqd1_12*ORjdqd1_23
    RLjdqd2_22 = s.dpt[2,23]*C11
    RLjdqd2_32 = s.dpt[2,23]*S11
    ORjdqd2_22 = -RLjdqd2_32*qd[11]
    ORjdqd2_32 = RLjdqd2_22*qd[11]
    Apqpjdqd2_22 = -ORjdqd2_32*qd[11]
    Apqpjdqd2_32 = ORjdqd2_22*qd[11]
    jdqd2 = Apqpjdqd1_23-Apqpjdqd2_22
    jdqd3 = Apqpjdqd1_33-Apqpjdqd2_32
    ROjdqd3_42 = S15*S19
    ROjdqd3_62 = C15*S19
    ROjdqd3_72 = S15*C19
    ROjdqd3_92 = C15*C19
    ROjdqd3_73 = ROjdqd3_72*C20+C15*S20
    ROjdqd3_83 = -S19*C20
    ROjdqd3_93 = ROjdqd3_92*C20-S15*S20
    RLjdqd3_12 = s.dpt[1,30]*C15
    RLjdqd3_32 = -s.dpt[1,30]*S15
    OMjdqd3_12 = qd[19]*C15
    OMjdqd3_32 = -qd[19]*S15
    ORjdqd3_12 = RLjdqd3_32*qd[15]
    ORjdqd3_32 = -RLjdqd3_12*qd[15]
    Ompqpjdqd3_12 = -qd[15]*qd[19]*S15
    Ompqpjdqd3_32 = -qd[15]*qd[19]*C15
    Apqpjdqd3_12 = ORjdqd3_32*qd[15]
    Apqpjdqd3_32 = -ORjdqd3_12*qd[15]
    OMjdqd3_13 = OMjdqd3_12+ROjdqd3_42*qd[20]
    OMjdqd3_23 = qd[15]+qd[20]*C19
    OMjdqd3_33 = OMjdqd3_32+ROjdqd3_62*qd[20]
    Ompqpjdqd3_13 = Ompqpjdqd3_12+qd[20]*(-OMjdqd3_32*C19+ROjdqd3_62*qd[15])
    Ompqpjdqd3_23 = qd[20]*(-OMjdqd3_12*ROjdqd3_62+OMjdqd3_32*ROjdqd3_42)
    Ompqpjdqd3_33 = Ompqpjdqd3_32+qd[20]*(OMjdqd3_12*C19-ROjdqd3_42*qd[15])
    RLjdqd3_14 = ROjdqd3_73*s.dpt[3,33]
    RLjdqd3_24 = ROjdqd3_83*s.dpt[3,33]
    RLjdqd3_34 = ROjdqd3_93*s.dpt[3,33]
    ORjdqd3_14 = OMjdqd3_23*RLjdqd3_34-OMjdqd3_33*RLjdqd3_24
    ORjdqd3_24 = -OMjdqd3_13*RLjdqd3_34+OMjdqd3_33*RLjdqd3_14
    ORjdqd3_34 = OMjdqd3_13*RLjdqd3_24-OMjdqd3_23*RLjdqd3_14
    Apqpjdqd3_14 = Apqpjdqd3_12+OMjdqd3_23*ORjdqd3_34-OMjdqd3_33*ORjdqd3_24+Ompqpjdqd3_23*RLjdqd3_34-Ompqpjdqd3_33* \
 	  RLjdqd3_24
    Apqpjdqd3_24 = -OMjdqd3_13*ORjdqd3_34+OMjdqd3_33*ORjdqd3_14-Ompqpjdqd3_13*RLjdqd3_34+Ompqpjdqd3_33*RLjdqd3_14
    Apqpjdqd3_34 = Apqpjdqd3_32+OMjdqd3_13*ORjdqd3_24-OMjdqd3_23*ORjdqd3_14+Ompqpjdqd3_13*RLjdqd3_24-Ompqpjdqd3_23* \
 	  RLjdqd3_14
    RLjdqd4_22 = s.dpt[2,25]*C11
    RLjdqd4_32 = s.dpt[2,25]*S11
    ORjdqd4_22 = -RLjdqd4_32*qd[11]
    ORjdqd4_32 = RLjdqd4_22*qd[11]
    Apqpjdqd4_22 = -ORjdqd4_32*qd[11]
    Apqpjdqd4_32 = ORjdqd4_22*qd[11]
    jdqd5 = Apqpjdqd3_24-Apqpjdqd4_22
    jdqd6 = Apqpjdqd3_34-Apqpjdqd4_32
    ROjdqd5_12 = C15*C16-S15*S16
    ROjdqd5_32 = -C15*S16-S15*C16
    ROjdqd5_72 = C15*S16+S15*C16
    ROjdqd5_92 = C15*C16-S15*S16
    ROjdqd5_43 = ROjdqd5_72*S17
    ROjdqd5_63 = ROjdqd5_92*S17
    ROjdqd5_73 = ROjdqd5_72*C17
    ROjdqd5_93 = ROjdqd5_92*C17
    ROjdqd5_74 = ROjdqd5_12*S18+ROjdqd5_73*C18
    ROjdqd5_84 = -S17*C18
    ROjdqd5_94 = ROjdqd5_32*S18+ROjdqd5_93*C18
    OMjdqd5_22 = qd[15]+qd[16]
    RLjdqd5_13 = ROjdqd5_12*s.dpt[1,31]
    RLjdqd5_33 = ROjdqd5_32*s.dpt[1,31]
    OMjdqd5_13 = ROjdqd5_12*qd[17]
    OMjdqd5_33 = ROjdqd5_32*qd[17]
    ORjdqd5_13 = OMjdqd5_22*RLjdqd5_33
    ORjdqd5_33 = -OMjdqd5_22*RLjdqd5_13
    Ompqpjdqd5_13 = OMjdqd5_22*ROjdqd5_32*qd[17]
    Ompqpjdqd5_33 = -OMjdqd5_22*ROjdqd5_12*qd[17]
    Apqpjdqd5_13 = OMjdqd5_22*ORjdqd5_33
    Apqpjdqd5_33 = -OMjdqd5_22*ORjdqd5_13
    OMjdqd5_14 = OMjdqd5_13+ROjdqd5_43*qd[18]
    OMjdqd5_24 = OMjdqd5_22+qd[18]*C17
    OMjdqd5_34 = OMjdqd5_33+ROjdqd5_63*qd[18]
    Ompqpjdqd5_14 = Ompqpjdqd5_13+qd[18]*(OMjdqd5_22*ROjdqd5_63-OMjdqd5_33*C17)
    Ompqpjdqd5_24 = qd[18]*(-OMjdqd5_13*ROjdqd5_63+OMjdqd5_33*ROjdqd5_43)
    Ompqpjdqd5_34 = Ompqpjdqd5_33+qd[18]*(OMjdqd5_13*C17-OMjdqd5_22*ROjdqd5_43)
    RLjdqd5_15 = ROjdqd5_74*s.dpt[3,32]
    RLjdqd5_25 = ROjdqd5_84*s.dpt[3,32]
    RLjdqd5_35 = ROjdqd5_94*s.dpt[3,32]
    ORjdqd5_15 = OMjdqd5_24*RLjdqd5_35-OMjdqd5_34*RLjdqd5_25
    ORjdqd5_25 = -OMjdqd5_14*RLjdqd5_35+OMjdqd5_34*RLjdqd5_15
    ORjdqd5_35 = OMjdqd5_14*RLjdqd5_25-OMjdqd5_24*RLjdqd5_15
    Apqpjdqd5_15 = Apqpjdqd5_13+OMjdqd5_24*ORjdqd5_35-OMjdqd5_34*ORjdqd5_25+Ompqpjdqd5_24*RLjdqd5_35-Ompqpjdqd5_34* \
 	  RLjdqd5_25
    Apqpjdqd5_25 = -OMjdqd5_14*ORjdqd5_35+OMjdqd5_34*ORjdqd5_15-Ompqpjdqd5_14*RLjdqd5_35+Ompqpjdqd5_34*RLjdqd5_15
    Apqpjdqd5_35 = Apqpjdqd5_33+OMjdqd5_14*ORjdqd5_25-OMjdqd5_24*ORjdqd5_15+Ompqpjdqd5_14*RLjdqd5_25-Ompqpjdqd5_24* \
 	  RLjdqd5_15
    RLjdqd6_22 = s.dpt[2,18]*C7
    RLjdqd6_32 = s.dpt[2,18]*S7
    ORjdqd6_22 = -RLjdqd6_32*qd[7]
    ORjdqd6_32 = RLjdqd6_22*qd[7]
    Apqpjdqd6_22 = -ORjdqd6_32*qd[7]
    Apqpjdqd6_32 = ORjdqd6_22*qd[7]
    jdqd8 = Apqpjdqd5_25-Apqpjdqd6_22
    jdqd9 = Apqpjdqd5_35-Apqpjdqd6_32
    ROjdqd7_52 = C21*C22-S21*S22
    ROjdqd7_62 = C21*S22+S21*C22
    ROjdqd7_82 = -C21*S22-S21*C22
    ROjdqd7_92 = C21*C22-S21*S22
    ROjdqd7_83 = ROjdqd7_82*C23
    ROjdqd7_93 = ROjdqd7_92*C23
    RLjdqd7_22 = s.dpt[2,34]*C21
    RLjdqd7_32 = s.dpt[2,34]*S21
    OMjdqd7_12 = qd[21]+qd[22]
    ORjdqd7_22 = -RLjdqd7_32*qd[21]
    ORjdqd7_32 = RLjdqd7_22*qd[21]
    Apqpjdqd7_22 = -ORjdqd7_32*qd[21]
    Apqpjdqd7_32 = ORjdqd7_22*qd[21]
    OMjdqd7_23 = ROjdqd7_52*qd[23]
    OMjdqd7_33 = ROjdqd7_62*qd[23]
    Ompqpjdqd7_23 = -OMjdqd7_12*ROjdqd7_62*qd[23]
    Ompqpjdqd7_33 = OMjdqd7_12*ROjdqd7_52*qd[23]
    OMjdqd7_14 = OMjdqd7_12+qd[24]*S23
    OMjdqd7_24 = OMjdqd7_23+ROjdqd7_83*qd[24]
    OMjdqd7_34 = OMjdqd7_33+ROjdqd7_93*qd[24]
    Ompqpjdqd7_14 = qd[24]*(OMjdqd7_23*ROjdqd7_93-OMjdqd7_33*ROjdqd7_83)
    Ompqpjdqd7_24 = Ompqpjdqd7_23+qd[24]*(-OMjdqd7_12*ROjdqd7_93+OMjdqd7_33*S23)
    Ompqpjdqd7_34 = Ompqpjdqd7_33+qd[24]*(OMjdqd7_12*ROjdqd7_83-OMjdqd7_23*S23)
    RLjdqd7_15 = s.dpt[3,35]*S23
    RLjdqd7_25 = ROjdqd7_83*s.dpt[3,35]
    RLjdqd7_35 = ROjdqd7_93*s.dpt[3,35]
    ORjdqd7_15 = OMjdqd7_24*RLjdqd7_35-OMjdqd7_34*RLjdqd7_25
    ORjdqd7_25 = -OMjdqd7_14*RLjdqd7_35+OMjdqd7_34*RLjdqd7_15
    ORjdqd7_35 = OMjdqd7_14*RLjdqd7_25-OMjdqd7_24*RLjdqd7_15
    Apqpjdqd7_15 = OMjdqd7_24*ORjdqd7_35-OMjdqd7_34*ORjdqd7_25+Ompqpjdqd7_24*RLjdqd7_35-Ompqpjdqd7_34*RLjdqd7_25
    Apqpjdqd7_25 = Apqpjdqd7_22-OMjdqd7_14*ORjdqd7_35+OMjdqd7_34*ORjdqd7_15-Ompqpjdqd7_14*RLjdqd7_35+Ompqpjdqd7_34* \
 	  RLjdqd7_15
    Apqpjdqd7_35 = Apqpjdqd7_32+OMjdqd7_14*ORjdqd7_25-OMjdqd7_24*ORjdqd7_15+Ompqpjdqd7_14*RLjdqd7_25-Ompqpjdqd7_24* \
 	  RLjdqd7_15
    RLjdqd8_22 = s.dpt[2,39]*C26
    RLjdqd8_32 = s.dpt[2,39]*S26
    ORjdqd8_22 = -RLjdqd8_32*qd[26]
    ORjdqd8_32 = RLjdqd8_22*qd[26]
    Apqpjdqd8_22 = -ORjdqd8_32*qd[26]
    Apqpjdqd8_32 = ORjdqd8_22*qd[26]
    jdqd11 = Apqpjdqd7_25-Apqpjdqd8_22
    jdqd12 = Apqpjdqd7_35-Apqpjdqd8_32
    ROjdqd9_42 = S33*S37
    ROjdqd9_62 = C33*S37
    ROjdqd9_72 = S33*C37
    ROjdqd9_92 = C33*C37
    ROjdqd9_73 = ROjdqd9_72*C38+C33*S38
    ROjdqd9_83 = -S37*C38
    ROjdqd9_93 = ROjdqd9_92*C38-S33*S38
    RLjdqd9_12 = s.dpt[1,50]*C33
    RLjdqd9_32 = -s.dpt[1,50]*S33
    OMjdqd9_12 = qd[37]*C33
    OMjdqd9_32 = -qd[37]*S33
    ORjdqd9_12 = RLjdqd9_32*qd[33]
    ORjdqd9_32 = -RLjdqd9_12*qd[33]
    Ompqpjdqd9_12 = -qd[33]*qd[37]*S33
    Ompqpjdqd9_32 = -qd[33]*qd[37]*C33
    Apqpjdqd9_12 = ORjdqd9_32*qd[33]
    Apqpjdqd9_32 = -ORjdqd9_12*qd[33]
    OMjdqd9_13 = OMjdqd9_12+ROjdqd9_42*qd[38]
    OMjdqd9_23 = qd[33]+qd[38]*C37
    OMjdqd9_33 = OMjdqd9_32+ROjdqd9_62*qd[38]
    Ompqpjdqd9_13 = Ompqpjdqd9_12+qd[38]*(-OMjdqd9_32*C37+ROjdqd9_62*qd[33])
    Ompqpjdqd9_23 = qd[38]*(-OMjdqd9_12*ROjdqd9_62+OMjdqd9_32*ROjdqd9_42)
    Ompqpjdqd9_33 = Ompqpjdqd9_32+qd[38]*(OMjdqd9_12*C37-ROjdqd9_42*qd[33])
    RLjdqd9_14 = ROjdqd9_73*s.dpt[3,53]
    RLjdqd9_24 = ROjdqd9_83*s.dpt[3,53]
    RLjdqd9_34 = ROjdqd9_93*s.dpt[3,53]
    ORjdqd9_14 = OMjdqd9_23*RLjdqd9_34-OMjdqd9_33*RLjdqd9_24
    ORjdqd9_24 = -OMjdqd9_13*RLjdqd9_34+OMjdqd9_33*RLjdqd9_14
    ORjdqd9_34 = OMjdqd9_13*RLjdqd9_24-OMjdqd9_23*RLjdqd9_14
    Apqpjdqd9_14 = Apqpjdqd9_12+OMjdqd9_23*ORjdqd9_34-OMjdqd9_33*ORjdqd9_24+Ompqpjdqd9_23*RLjdqd9_34-Ompqpjdqd9_33* \
 	  RLjdqd9_24
    Apqpjdqd9_24 = -OMjdqd9_13*ORjdqd9_34+OMjdqd9_33*ORjdqd9_14-Ompqpjdqd9_13*RLjdqd9_34+Ompqpjdqd9_33*RLjdqd9_14
    Apqpjdqd9_34 = Apqpjdqd9_32+OMjdqd9_13*ORjdqd9_24-OMjdqd9_23*ORjdqd9_14+Ompqpjdqd9_13*RLjdqd9_24-Ompqpjdqd9_23* \
 	  RLjdqd9_14
    RLjdqd10_22 = s.dpt[2,41]*C26
    RLjdqd10_32 = s.dpt[2,41]*S26
    ORjdqd10_22 = -RLjdqd10_32*qd[26]
    ORjdqd10_32 = RLjdqd10_22*qd[26]
    Apqpjdqd10_22 = -ORjdqd10_32*qd[26]
    Apqpjdqd10_32 = ORjdqd10_22*qd[26]
    jdqd14 = -Apqpjdqd10_22+Apqpjdqd9_24
    jdqd15 = -Apqpjdqd10_32+Apqpjdqd9_34
    ROjdqd11_12 = C33*C34-S33*S34
    ROjdqd11_32 = -C33*S34-S33*C34
    ROjdqd11_72 = C33*S34+S33*C34
    ROjdqd11_92 = C33*C34-S33*S34
    ROjdqd11_43 = ROjdqd11_72*S35
    ROjdqd11_63 = ROjdqd11_92*S35
    ROjdqd11_73 = ROjdqd11_72*C35
    ROjdqd11_93 = ROjdqd11_92*C35
    ROjdqd11_74 = ROjdqd11_12*S36+ROjdqd11_73*C36
    ROjdqd11_84 = -S35*C36
    ROjdqd11_94 = ROjdqd11_32*S36+ROjdqd11_93*C36
    OMjdqd11_22 = qd[33]+qd[34]
    RLjdqd11_13 = ROjdqd11_12*s.dpt[1,51]
    RLjdqd11_33 = ROjdqd11_32*s.dpt[1,51]
    OMjdqd11_13 = ROjdqd11_12*qd[35]
    OMjdqd11_33 = ROjdqd11_32*qd[35]
    ORjdqd11_13 = OMjdqd11_22*RLjdqd11_33
    ORjdqd11_33 = -OMjdqd11_22*RLjdqd11_13
    Ompqpjdqd11_13 = OMjdqd11_22*ROjdqd11_32*qd[35]
    Ompqpjdqd11_33 = -OMjdqd11_22*ROjdqd11_12*qd[35]
    Apqpjdqd11_13 = OMjdqd11_22*ORjdqd11_33
    Apqpjdqd11_33 = -OMjdqd11_22*ORjdqd11_13
    OMjdqd11_14 = OMjdqd11_13+ROjdqd11_43*qd[36]
    OMjdqd11_24 = OMjdqd11_22+qd[36]*C35
    OMjdqd11_34 = OMjdqd11_33+ROjdqd11_63*qd[36]
    Ompqpjdqd11_14 = Ompqpjdqd11_13+qd[36]*(OMjdqd11_22*ROjdqd11_63-OMjdqd11_33*C35)
    Ompqpjdqd11_24 = qd[36]*(-OMjdqd11_13*ROjdqd11_63+OMjdqd11_33*ROjdqd11_43)
    Ompqpjdqd11_34 = Ompqpjdqd11_33+qd[36]*(OMjdqd11_13*C35-OMjdqd11_22*ROjdqd11_43)
    RLjdqd11_15 = ROjdqd11_74*s.dpt[3,52]
    RLjdqd11_25 = ROjdqd11_84*s.dpt[3,52]
    RLjdqd11_35 = ROjdqd11_94*s.dpt[3,52]
    ORjdqd11_15 = OMjdqd11_24*RLjdqd11_35-OMjdqd11_34*RLjdqd11_25
    ORjdqd11_25 = -OMjdqd11_14*RLjdqd11_35+OMjdqd11_34*RLjdqd11_15
    ORjdqd11_35 = OMjdqd11_14*RLjdqd11_25-OMjdqd11_24*RLjdqd11_15
    Apqpjdqd11_15 = Apqpjdqd11_13+OMjdqd11_24*ORjdqd11_35-OMjdqd11_34*ORjdqd11_25+Ompqpjdqd11_24*RLjdqd11_35- \
 	  Ompqpjdqd11_34*RLjdqd11_25
    Apqpjdqd11_25 = -OMjdqd11_14*ORjdqd11_35+OMjdqd11_34*ORjdqd11_15-Ompqpjdqd11_14*RLjdqd11_35+Ompqpjdqd11_34* \
 	  RLjdqd11_15
    Apqpjdqd11_35 = Apqpjdqd11_33+OMjdqd11_14*ORjdqd11_25-OMjdqd11_24*ORjdqd11_15+Ompqpjdqd11_14*RLjdqd11_25- \
 	  Ompqpjdqd11_24*RLjdqd11_15
    RLjdqd12_22 = s.dpt[2,49]*C32
    RLjdqd12_32 = s.dpt[2,49]*S32
    ORjdqd12_22 = -RLjdqd12_32*qd[32]
    ORjdqd12_32 = RLjdqd12_22*qd[32]
    Apqpjdqd12_22 = -ORjdqd12_32*qd[32]
    Apqpjdqd12_32 = ORjdqd12_22*qd[32]
    jdqd17 = Apqpjdqd11_25-Apqpjdqd12_22
    jdqd18 = Apqpjdqd11_35-Apqpjdqd12_32
    ROjdqd13_52 = C21*C22-S21*S22
    ROjdqd13_62 = C21*S22+S21*C22
    ROjdqd13_82 = -C21*S22-S21*C22
    ROjdqd13_92 = C21*C22-S21*S22
    ROjdqd13_23 = -ROjdqd13_82*S23
    ROjdqd13_33 = -ROjdqd13_92*S23
    ROjdqd13_83 = ROjdqd13_82*C23
    ROjdqd13_93 = ROjdqd13_92*C23
    ROjdqd13_14 = C23*C24
    ROjdqd13_24 = ROjdqd13_23*C24+ROjdqd13_52*S24
    ROjdqd13_34 = ROjdqd13_33*C24+ROjdqd13_62*S24
    ROjdqd13_44 = -C23*S24
    ROjdqd13_54 = -ROjdqd13_23*S24+ROjdqd13_52*C24
    ROjdqd13_64 = -ROjdqd13_33*S24+ROjdqd13_62*C24
    RLjdqd13_22 = s.dpt[2,34]*C21
    RLjdqd13_32 = s.dpt[2,34]*S21
    OMjdqd13_12 = qd[21]+qd[22]
    ORjdqd13_22 = -RLjdqd13_32*qd[21]
    ORjdqd13_32 = RLjdqd13_22*qd[21]
    Apqpjdqd13_22 = -ORjdqd13_32*qd[21]
    Apqpjdqd13_32 = ORjdqd13_22*qd[21]
    OMjdqd13_23 = ROjdqd13_52*qd[23]
    OMjdqd13_33 = ROjdqd13_62*qd[23]
    Ompqpjdqd13_23 = -OMjdqd13_12*ROjdqd13_62*qd[23]
    Ompqpjdqd13_33 = OMjdqd13_12*ROjdqd13_52*qd[23]
    OMjdqd13_14 = OMjdqd13_12+qd[24]*S23
    OMjdqd13_24 = OMjdqd13_23+ROjdqd13_83*qd[24]
    OMjdqd13_34 = OMjdqd13_33+ROjdqd13_93*qd[24]
    Ompqpjdqd13_14 = qd[24]*(OMjdqd13_23*ROjdqd13_93-OMjdqd13_33*ROjdqd13_83)
    Ompqpjdqd13_24 = Ompqpjdqd13_23+qd[24]*(-OMjdqd13_12*ROjdqd13_93+OMjdqd13_33*S23)
    Ompqpjdqd13_34 = Ompqpjdqd13_33+qd[24]*(OMjdqd13_12*ROjdqd13_83-OMjdqd13_23*S23)
    RLjdqd13_15 = ROjdqd13_14*s.dpt[1,37]+ROjdqd13_44*s.dpt[2,37]+s.dpt[3,37]*S23
    RLjdqd13_25 = ROjdqd13_24*s.dpt[1,37]+ROjdqd13_54*s.dpt[2,37]+ROjdqd13_83*s.dpt[3,37]
    RLjdqd13_35 = ROjdqd13_34*s.dpt[1,37]+ROjdqd13_64*s.dpt[2,37]+ROjdqd13_93*s.dpt[3,37]
    ORjdqd13_15 = OMjdqd13_24*RLjdqd13_35-OMjdqd13_34*RLjdqd13_25
    ORjdqd13_25 = -OMjdqd13_14*RLjdqd13_35+OMjdqd13_34*RLjdqd13_15
    ORjdqd13_35 = OMjdqd13_14*RLjdqd13_25-OMjdqd13_24*RLjdqd13_15
    Apqpjdqd13_15 = OMjdqd13_24*ORjdqd13_35-OMjdqd13_34*ORjdqd13_25+Ompqpjdqd13_24*RLjdqd13_35-Ompqpjdqd13_34* \
 	  RLjdqd13_25
    Apqpjdqd13_25 = Apqpjdqd13_22-OMjdqd13_14*ORjdqd13_35+OMjdqd13_34*ORjdqd13_15-Ompqpjdqd13_14*RLjdqd13_35+ \
 	  Ompqpjdqd13_34*RLjdqd13_15
    Apqpjdqd13_35 = Apqpjdqd13_32+OMjdqd13_14*ORjdqd13_25-OMjdqd13_24*ORjdqd13_15+Ompqpjdqd13_14*RLjdqd13_25- \
 	  Ompqpjdqd13_24*RLjdqd13_15
    ROjdqd14_23 = C42*S43
    ROjdqd14_33 = S42*S43
    ROjdqd14_53 = C42*C43
    ROjdqd14_63 = S42*C43
    OMjdqd14_23 = -qd[43]*S42
    OMjdqd14_33 = qd[43]*C42
    Ompqpjdqd14_23 = -qd[42]*qd[43]*C42
    Ompqpjdqd14_33 = -qd[42]*qd[43]*S42
    RLjdqd14_14 = s.dpt[1,57]*C43-s.dpt[2,57]*S43
    RLjdqd14_24 = ROjdqd14_23*s.dpt[1,57]+ROjdqd14_53*s.dpt[2,57]
    RLjdqd14_34 = ROjdqd14_33*s.dpt[1,57]+ROjdqd14_63*s.dpt[2,57]
    ORjdqd14_14 = OMjdqd14_23*RLjdqd14_34-OMjdqd14_33*RLjdqd14_24
    ORjdqd14_24 = OMjdqd14_33*RLjdqd14_14-RLjdqd14_34*qd[42]
    ORjdqd14_34 = -OMjdqd14_23*RLjdqd14_14+RLjdqd14_24*qd[42]
    Apqpjdqd14_14 = OMjdqd14_23*ORjdqd14_34-OMjdqd14_33*ORjdqd14_24+Ompqpjdqd14_23*RLjdqd14_34-Ompqpjdqd14_33* \
 	  RLjdqd14_24
    Apqpjdqd14_24 = OMjdqd14_33*ORjdqd14_14-ORjdqd14_34*qd[42]+Ompqpjdqd14_33*RLjdqd14_14
    Apqpjdqd14_34 = -OMjdqd14_23*ORjdqd14_14+ORjdqd14_24*qd[42]-Ompqpjdqd14_23*RLjdqd14_14
    jdqd19 = Apqpjdqd13_15-Apqpjdqd14_14
    jdqd20 = Apqpjdqd13_25-Apqpjdqd14_24
    jdqd21 = Apqpjdqd13_35-Apqpjdqd14_34
    ROjdqd15_52 = C27*C28-S27*S28
    ROjdqd15_62 = C27*S28+S27*C28
    ROjdqd15_82 = -C27*S28-S27*C28
    ROjdqd15_92 = C27*C28-S27*S28
    ROjdqd15_83 = ROjdqd15_82*C29
    ROjdqd15_93 = ROjdqd15_92*C29
    RLjdqd15_22 = s.dpt[2,42]*C27
    RLjdqd15_32 = s.dpt[2,42]*S27
    OMjdqd15_12 = qd[27]+qd[28]
    ORjdqd15_22 = -RLjdqd15_32*qd[27]
    ORjdqd15_32 = RLjdqd15_22*qd[27]
    Apqpjdqd15_22 = -ORjdqd15_32*qd[27]
    Apqpjdqd15_32 = ORjdqd15_22*qd[27]
    OMjdqd15_23 = ROjdqd15_52*qd[29]
    OMjdqd15_33 = ROjdqd15_62*qd[29]
    Ompqpjdqd15_23 = -OMjdqd15_12*ROjdqd15_62*qd[29]
    Ompqpjdqd15_33 = OMjdqd15_12*ROjdqd15_52*qd[29]
    OMjdqd15_14 = OMjdqd15_12+qd[30]*S29
    OMjdqd15_24 = OMjdqd15_23+ROjdqd15_83*qd[30]
    OMjdqd15_34 = OMjdqd15_33+ROjdqd15_93*qd[30]
    Ompqpjdqd15_14 = qd[30]*(OMjdqd15_23*ROjdqd15_93-OMjdqd15_33*ROjdqd15_83)
    Ompqpjdqd15_24 = Ompqpjdqd15_23+qd[30]*(-OMjdqd15_12*ROjdqd15_93+OMjdqd15_33*S29)
    Ompqpjdqd15_34 = Ompqpjdqd15_33+qd[30]*(OMjdqd15_12*ROjdqd15_83-OMjdqd15_23*S29)
    RLjdqd15_15 = s.dpt[3,43]*S29
    RLjdqd15_25 = ROjdqd15_83*s.dpt[3,43]
    RLjdqd15_35 = ROjdqd15_93*s.dpt[3,43]
    ORjdqd15_15 = OMjdqd15_24*RLjdqd15_35-OMjdqd15_34*RLjdqd15_25
    ORjdqd15_25 = -OMjdqd15_14*RLjdqd15_35+OMjdqd15_34*RLjdqd15_15
    ORjdqd15_35 = OMjdqd15_14*RLjdqd15_25-OMjdqd15_24*RLjdqd15_15
    Apqpjdqd15_15 = OMjdqd15_24*ORjdqd15_35-OMjdqd15_34*ORjdqd15_25+Ompqpjdqd15_24*RLjdqd15_35-Ompqpjdqd15_34* \
 	  RLjdqd15_25
    Apqpjdqd15_25 = Apqpjdqd15_22-OMjdqd15_14*ORjdqd15_35+OMjdqd15_34*ORjdqd15_15-Ompqpjdqd15_14*RLjdqd15_35+ \
 	  Ompqpjdqd15_34*RLjdqd15_15
    Apqpjdqd15_35 = Apqpjdqd15_32+OMjdqd15_14*ORjdqd15_25-OMjdqd15_24*ORjdqd15_15+Ompqpjdqd15_14*RLjdqd15_25- \
 	  Ompqpjdqd15_24*RLjdqd15_15
    RLjdqd16_22 = s.dpt[2,47]*C32
    RLjdqd16_32 = s.dpt[2,47]*S32
    ORjdqd16_22 = -RLjdqd16_32*qd[32]
    ORjdqd16_32 = RLjdqd16_22*qd[32]
    Apqpjdqd16_22 = -ORjdqd16_32*qd[32]
    Apqpjdqd16_32 = ORjdqd16_22*qd[32]
    jdqd23 = Apqpjdqd15_25-Apqpjdqd16_22
    jdqd24 = Apqpjdqd15_35-Apqpjdqd16_32
    ROjdqd17_82 = -C8*S9-S8*C9
    ROjdqd17_92 = C8*C9-S8*S9
    RLjdqd17_22 = s.dpt[2,19]*C8
    RLjdqd17_32 = s.dpt[2,19]*S8
    OMjdqd17_12 = qd[8]+qd[9]
    ORjdqd17_22 = -RLjdqd17_32*qd[8]
    ORjdqd17_32 = RLjdqd17_22*qd[8]
    Apqpjdqd17_22 = -ORjdqd17_32*qd[8]
    Apqpjdqd17_32 = ORjdqd17_22*qd[8]
    RLjdqd17_23 = ROjdqd17_82*s.dpt[3,20]
    RLjdqd17_33 = ROjdqd17_92*s.dpt[3,20]
    ORjdqd17_23 = -OMjdqd17_12*RLjdqd17_33
    ORjdqd17_33 = OMjdqd17_12*RLjdqd17_23
    Apqpjdqd17_23 = Apqpjdqd17_22-OMjdqd17_12*ORjdqd17_33
    Apqpjdqd17_33 = Apqpjdqd17_32+OMjdqd17_12*ORjdqd17_23
    RLjdqd18_22 = s.dpt[2,16]*C7
    RLjdqd18_32 = s.dpt[2,16]*S7
    ORjdqd18_22 = -RLjdqd18_32*qd[7]
    ORjdqd18_32 = RLjdqd18_22*qd[7]
    Apqpjdqd18_22 = -ORjdqd18_32*qd[7]
    Apqpjdqd18_32 = ORjdqd18_22*qd[7]
    jdqd26 = Apqpjdqd17_23-Apqpjdqd18_22
    jdqd27 = Apqpjdqd17_33-Apqpjdqd18_32
    ROjdqd19_52 = C27*C28-S27*S28
    ROjdqd19_62 = C27*S28+S27*C28
    ROjdqd19_82 = -C27*S28-S27*C28
    ROjdqd19_92 = C27*C28-S27*S28
    ROjdqd19_23 = -ROjdqd19_82*S29
    ROjdqd19_33 = -ROjdqd19_92*S29
    ROjdqd19_83 = ROjdqd19_82*C29
    ROjdqd19_93 = ROjdqd19_92*C29
    ROjdqd19_14 = C29*C30
    ROjdqd19_24 = ROjdqd19_23*C30+ROjdqd19_52*S30
    ROjdqd19_34 = ROjdqd19_33*C30+ROjdqd19_62*S30
    ROjdqd19_44 = -C29*S30
    ROjdqd19_54 = -ROjdqd19_23*S30+ROjdqd19_52*C30
    ROjdqd19_64 = -ROjdqd19_33*S30+ROjdqd19_62*C30
    RLjdqd19_22 = s.dpt[2,42]*C27
    RLjdqd19_32 = s.dpt[2,42]*S27
    OMjdqd19_12 = qd[27]+qd[28]
    ORjdqd19_22 = -RLjdqd19_32*qd[27]
    ORjdqd19_32 = RLjdqd19_22*qd[27]
    Apqpjdqd19_22 = -ORjdqd19_32*qd[27]
    Apqpjdqd19_32 = ORjdqd19_22*qd[27]
    OMjdqd19_23 = ROjdqd19_52*qd[29]
    OMjdqd19_33 = ROjdqd19_62*qd[29]
    Ompqpjdqd19_23 = -OMjdqd19_12*ROjdqd19_62*qd[29]
    Ompqpjdqd19_33 = OMjdqd19_12*ROjdqd19_52*qd[29]
    OMjdqd19_14 = OMjdqd19_12+qd[30]*S29
    OMjdqd19_24 = OMjdqd19_23+ROjdqd19_83*qd[30]
    OMjdqd19_34 = OMjdqd19_33+ROjdqd19_93*qd[30]
    Ompqpjdqd19_14 = qd[30]*(OMjdqd19_23*ROjdqd19_93-OMjdqd19_33*ROjdqd19_83)
    Ompqpjdqd19_24 = Ompqpjdqd19_23+qd[30]*(-OMjdqd19_12*ROjdqd19_93+OMjdqd19_33*S29)
    Ompqpjdqd19_34 = Ompqpjdqd19_33+qd[30]*(OMjdqd19_12*ROjdqd19_83-OMjdqd19_23*S29)
    RLjdqd19_15 = ROjdqd19_14*s.dpt[1,45]+ROjdqd19_44*s.dpt[2,45]+s.dpt[3,45]*S29
    RLjdqd19_25 = ROjdqd19_24*s.dpt[1,45]+ROjdqd19_54*s.dpt[2,45]+ROjdqd19_83*s.dpt[3,45]
    RLjdqd19_35 = ROjdqd19_34*s.dpt[1,45]+ROjdqd19_64*s.dpt[2,45]+ROjdqd19_93*s.dpt[3,45]
    ORjdqd19_15 = OMjdqd19_24*RLjdqd19_35-OMjdqd19_34*RLjdqd19_25
    ORjdqd19_25 = -OMjdqd19_14*RLjdqd19_35+OMjdqd19_34*RLjdqd19_15
    ORjdqd19_35 = OMjdqd19_14*RLjdqd19_25-OMjdqd19_24*RLjdqd19_15
    Apqpjdqd19_15 = OMjdqd19_24*ORjdqd19_35-OMjdqd19_34*ORjdqd19_25+Ompqpjdqd19_24*RLjdqd19_35-Ompqpjdqd19_34* \
 	  RLjdqd19_25
    Apqpjdqd19_25 = Apqpjdqd19_22-OMjdqd19_14*ORjdqd19_35+OMjdqd19_34*ORjdqd19_15-Ompqpjdqd19_14*RLjdqd19_35+ \
 	  Ompqpjdqd19_34*RLjdqd19_15
    Apqpjdqd19_35 = Apqpjdqd19_32+OMjdqd19_14*ORjdqd19_25-OMjdqd19_24*ORjdqd19_15+Ompqpjdqd19_14*RLjdqd19_25- \
 	  Ompqpjdqd19_24*RLjdqd19_15
    ROjdqd20_23 = C40*S41
    ROjdqd20_33 = S40*S41
    ROjdqd20_53 = C40*C41
    ROjdqd20_63 = S40*C41
    OMjdqd20_23 = -qd[41]*S40
    OMjdqd20_33 = qd[41]*C40
    Ompqpjdqd20_23 = -qd[40]*qd[41]*C40
    Ompqpjdqd20_33 = -qd[40]*qd[41]*S40
    RLjdqd20_14 = s.dpt[1,56]*C41-s.dpt[2,56]*S41
    RLjdqd20_24 = ROjdqd20_23*s.dpt[1,56]+ROjdqd20_53*s.dpt[2,56]
    RLjdqd20_34 = ROjdqd20_33*s.dpt[1,56]+ROjdqd20_63*s.dpt[2,56]
    ORjdqd20_14 = OMjdqd20_23*RLjdqd20_34-OMjdqd20_33*RLjdqd20_24
    ORjdqd20_24 = OMjdqd20_33*RLjdqd20_14-RLjdqd20_34*qd[40]
    ORjdqd20_34 = -OMjdqd20_23*RLjdqd20_14+RLjdqd20_24*qd[40]
    Apqpjdqd20_14 = OMjdqd20_23*ORjdqd20_34-OMjdqd20_33*ORjdqd20_24+Ompqpjdqd20_23*RLjdqd20_34-Ompqpjdqd20_33* \
 	  RLjdqd20_24
    Apqpjdqd20_24 = OMjdqd20_33*ORjdqd20_14-ORjdqd20_34*qd[40]+Ompqpjdqd20_33*RLjdqd20_14
    Apqpjdqd20_34 = -OMjdqd20_23*ORjdqd20_14+ORjdqd20_24*qd[40]-Ompqpjdqd20_23*RLjdqd20_14
    jdqd28 = Apqpjdqd19_15-Apqpjdqd20_14
    jdqd29 = Apqpjdqd19_25-Apqpjdqd20_24
    jdqd30 = Apqpjdqd19_35-Apqpjdqd20_34
    Jdqd[1] = jdqd2
    Jdqd[2] = jdqd3
    Jdqd[3] = Apqpjdqd3_14
    Jdqd[4] = jdqd5
    Jdqd[5] = jdqd6
    Jdqd[6] = Apqpjdqd5_15
    Jdqd[7] = jdqd8
    Jdqd[8] = jdqd9
    Jdqd[9] = Apqpjdqd7_15
    Jdqd[10] = jdqd11
    Jdqd[11] = jdqd12
    Jdqd[12] = Apqpjdqd9_14
    Jdqd[13] = jdqd14
    Jdqd[14] = jdqd15
    Jdqd[15] = Apqpjdqd11_15
    Jdqd[16] = jdqd17
    Jdqd[17] = jdqd18
    Jdqd[18] = jdqd19
    Jdqd[19] = jdqd20
    Jdqd[20] = jdqd21
    Jdqd[21] = Apqpjdqd15_15
    Jdqd[22] = jdqd23
    Jdqd[23] = jdqd24
    Jdqd[24] = jdqd26
    Jdqd[25] = jdqd27
    Jdqd[26] = jdqd28
    Jdqd[27] = jdqd29
    Jdqd[28] = jdqd30

# Number of continuation lines = 1


