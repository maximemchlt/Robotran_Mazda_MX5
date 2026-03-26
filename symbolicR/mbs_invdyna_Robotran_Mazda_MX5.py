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
#	==> Function: F2 - Recursive Inverse Dynamics of tree-like MBS
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos

def invdyna(phi,s,tsim):
    Qq = phi  # compatibility with symbolic generation
    q = s.q
    qd = s.qd
    qdd = s.qdd
 
# Trigonometric functions

    S4 = sin(q[4])
    C4 = cos(q[4])
    S5 = sin(q[5])
    C5 = cos(q[5])
    S6 = sin(q[6])
    C6 = cos(q[6])
    S7 = sin(q[7])
    C7 = cos(q[7])
    S8 = sin(q[8])
    C8 = cos(q[8])
    S9 = sin(q[9])
    C9 = cos(q[9])
    S10 = sin(q[10])
    C10 = cos(q[10])
    S11 = sin(q[11])
    C11 = cos(q[11])
    S12 = sin(q[12])
    C12 = cos(q[12])
 
# Augmented Joint Position Vectors

 
# Forward Kinematics

    ALPHA24 = qdd[2]*C4+qdd[3]*S4
    ALPHA34 = -qdd[2]*S4+qdd[3]*C4
    OM15 = qd[4]*C5
    OM35 = qd[4]*S5
    OMp15 = -qd[4]*qd[5]*S5+qdd[4]*C5
    OMp35 = qd[4]*qd[5]*C5+qdd[4]*S5
    ALPHA15 = qdd[1]*C5-ALPHA34*S5
    ALPHA35 = qdd[1]*S5+ALPHA34*C5
    OM16 = qd[5]*S6+OM15*C6
    OM26 = qd[5]*C6-OM15*S6
    OM36 = qd[6]+OM35
    OMp16 = C6*(OMp15+qd[5]*qd[6])+S6*(qdd[5]-qd[6]*OM15)
    OMp26 = C6*(qdd[5]-qd[6]*OM15)-S6*(OMp15+qd[5]*qd[6])
    OMp36 = qdd[6]+OMp35
    BS16 = -OM26*OM26-OM36*OM36
    BS26 = OM16*OM26
    BS36 = OM16*OM36
    BS56 = -OM16*OM16-OM36*OM36
    BS66 = OM26*OM36
    BS96 = -OM16*OM16-OM26*OM26
    BETA26 = BS26-OMp36
    BETA36 = BS36+OMp26
    BETA46 = BS26+OMp36
    BETA66 = BS66-OMp16
    BETA76 = BS36-OMp26
    BETA86 = BS66+OMp16
    ALPHA16 = ALPHA15*C6+ALPHA24*S6
    ALPHA26 = -ALPHA15*S6+ALPHA24*C6
    OM17 = qd[7]+OM16
    OM27 = OM26*C7+OM36*S7
    OM37 = -OM26*S7+OM36*C7
    OMp17 = qdd[7]+OMp16
    OMp27 = C7*(OMp26+qd[7]*OM36)+S7*(OMp36-qd[7]*OM26)
    OMp37 = C7*(OMp36-qd[7]*OM26)-S7*(OMp26+qd[7]*OM36)
    BS27 = OM17*OM27
    BS57 = -OM17*OM17-OM37*OM37
    BS67 = OM27*OM37
    BETA27 = BS27-OMp37
    BETA87 = BS67+OMp17
    ALPHA17 = ALPHA16+BETA26*s.dpt[2,1]+BETA36*s.dpt[3,1]+BS16*s.dpt[1,1]
    ALPHA27 = C7*(ALPHA26+BETA46*s.dpt[1,1]+BETA66*s.dpt[3,1]+BS56*s.dpt[2,1])+S7*(ALPHA35+BETA76*s.dpt[1,1]+BETA86* \
 	  s.dpt[2,1]+BS96*s.dpt[3,1])
    ALPHA37 = C7*(ALPHA35+BETA76*s.dpt[1,1]+BETA86*s.dpt[2,1]+BS96*s.dpt[3,1])-S7*(ALPHA26+BETA46*s.dpt[1,1]+BETA66* \
 	  s.dpt[3,1]+BS56*s.dpt[2,1])
    OM18 = qd[8]+OM16
    OM28 = OM26*C8+OM36*S8
    OM38 = -OM26*S8+OM36*C8
    OMp18 = qdd[8]+OMp16
    OMp28 = C8*(OMp26+qd[8]*OM36)+S8*(OMp36-qd[8]*OM26)
    OMp38 = C8*(OMp36-qd[8]*OM26)-S8*(OMp26+qd[8]*OM36)
    BS28 = OM18*OM28
    BS58 = -OM18*OM18-OM38*OM38
    BS68 = OM28*OM38
    BETA28 = BS28-OMp38
    BETA88 = BS68+OMp18
    ALPHA18 = ALPHA16+BETA26*s.dpt[2,2]+BETA36*s.dpt[3,2]+BS16*s.dpt[1,2]
    ALPHA28 = C8*(ALPHA26+BETA46*s.dpt[1,2]+BETA66*s.dpt[3,2]+BS56*s.dpt[2,2])+S8*(ALPHA35+BETA76*s.dpt[1,2]+BETA86* \
 	  s.dpt[2,2]+BS96*s.dpt[3,2])
    ALPHA38 = C8*(ALPHA35+BETA76*s.dpt[1,2]+BETA86*s.dpt[2,2]+BS96*s.dpt[3,2])-S8*(ALPHA26+BETA46*s.dpt[1,2]+BETA66* \
 	  s.dpt[3,2]+BS56*s.dpt[2,2])
    OM19 = qd[9]+OM18
    OM29 = OM28*C9+OM38*S9
    OM39 = -OM28*S9+OM38*C9
    OMp19 = qdd[9]+OMp18
    OMp29 = C9*(OMp28+qd[9]*OM38)+S9*(OMp38-qd[9]*OM28)
    OMp39 = C9*(OMp38-qd[9]*OM28)-S9*(OMp28+qd[9]*OM38)
    BS39 = OM19*OM39
    BS69 = OM29*OM39
    BS99 = -OM19*OM19-OM29*OM29
    BETA39 = BS39+OMp29
    BETA69 = BS69-OMp19
    ALPHA19 = ALPHA18+BETA28*s.dpt[2,9]
    ALPHA29 = C9*(ALPHA28+BS58*s.dpt[2,9])+S9*(ALPHA38+BETA88*s.dpt[2,9])
    ALPHA39 = C9*(ALPHA38+BETA88*s.dpt[2,9])-S9*(ALPHA28+BS58*s.dpt[2,9])
    OM110 = qd[10]+OM16
    OM210 = OM26*C10+OM36*S10
    OM310 = -OM26*S10+OM36*C10
    OMp110 = qdd[10]+OMp16
    OMp210 = C10*(OMp26+qd[10]*OM36)+S10*(OMp36-qd[10]*OM26)
    OMp310 = C10*(OMp36-qd[10]*OM26)-S10*(OMp26+qd[10]*OM36)
    BS210 = OM110*OM210
    BS510 = -OM110*OM110-OM310*OM310
    BS610 = OM210*OM310
    BETA210 = BS210-OMp310
    BETA810 = BS610+OMp110
    ALPHA110 = ALPHA16+BETA26*s.dpt[2,4]+BETA36*s.dpt[3,4]+BS16*s.dpt[1,4]
    ALPHA210 = C10*(ALPHA26+BETA46*s.dpt[1,4]+BETA66*s.dpt[3,4]+BS56*s.dpt[2,4])+S10*(ALPHA35+BETA76*s.dpt[1,4]+ \
 	  BETA86*s.dpt[2,4]+BS96*s.dpt[3,4])
    ALPHA310 = C10*(ALPHA35+BETA76*s.dpt[1,4]+BETA86*s.dpt[2,4]+BS96*s.dpt[3,4])-S10*(ALPHA26+BETA46*s.dpt[1,4]+ \
 	  BETA66*s.dpt[3,4]+BS56*s.dpt[2,4])
    OM111 = qd[11]+OM16
    OM211 = OM26*C11+OM36*S11
    OM311 = -OM26*S11+OM36*C11
    OMp111 = qdd[11]+OMp16
    OMp211 = C11*(OMp26+qd[11]*OM36)+S11*(OMp36-qd[11]*OM26)
    OMp311 = C11*(OMp36-qd[11]*OM26)-S11*(OMp26+qd[11]*OM36)
    BS211 = OM111*OM211
    BS511 = -OM111*OM111-OM311*OM311
    BS611 = OM211*OM311
    BETA211 = BS211-OMp311
    BETA811 = BS611+OMp111
    ALPHA111 = ALPHA16+BETA26*s.dpt[2,5]+BETA36*s.dpt[3,5]+BS16*s.dpt[1,5]
    ALPHA211 = C11*(ALPHA26+BETA46*s.dpt[1,5]+BETA66*s.dpt[3,5]+BS56*s.dpt[2,5])+S11*(ALPHA35+BETA76*s.dpt[1,5]+ \
 	  BETA86*s.dpt[2,5]+BS96*s.dpt[3,5])
    ALPHA311 = C11*(ALPHA35+BETA76*s.dpt[1,5]+BETA86*s.dpt[2,5]+BS96*s.dpt[3,5])-S11*(ALPHA26+BETA46*s.dpt[1,5]+ \
 	  BETA66*s.dpt[3,5]+BS56*s.dpt[2,5])
    OM112 = qd[12]+OM111
    OM212 = OM211*C12+OM311*S12
    OM312 = -OM211*S12+OM311*C12
    OMp112 = qdd[12]+OMp111
    OMp212 = C12*(OMp211+qd[12]*OM311)+S12*(OMp311-qd[12]*OM211)
    OMp312 = C12*(OMp311-qd[12]*OM211)-S12*(OMp211+qd[12]*OM311)
    BS312 = OM112*OM312
    BS612 = OM212*OM312
    BS912 = -OM112*OM112-OM212*OM212
    BETA312 = BS312+OMp212
    BETA612 = BS612-OMp112
    ALPHA112 = ALPHA111+BETA211*s.dpt[2,13]
    ALPHA212 = C12*(ALPHA211+BS511*s.dpt[2,13])+S12*(ALPHA311+BETA811*s.dpt[2,13])
    ALPHA312 = C12*(ALPHA311+BETA811*s.dpt[2,13])-S12*(ALPHA211+BS511*s.dpt[2,13])
 
# Backward Dynamics

    Fs112 = -s.frc[1,12]+s.m[12]*(ALPHA112+BETA312*s.l[3,12])
    Fs212 = -s.frc[2,12]+s.m[12]*(ALPHA212+BETA612*s.l[3,12])
    Fs312 = -s.frc[3,12]+s.m[12]*(ALPHA312+BS912*s.l[3,12])
    Cq112 = -s.trq[1,12]+s.In[1,12]*OMp112-s.In[5,12]*OM212*OM312+s.In[9,12]*OM212*OM312-Fs212*s.l[3,12]
    Cq212 = -s.trq[2,12]+s.In[1,12]*OM112*OM312+s.In[5,12]*OMp212-s.In[9,12]*OM112*OM312+Fs112*s.l[3,12]
    Cq312 = -s.trq[3,12]-s.In[1,12]*OM112*OM212+s.In[5,12]*OM112*OM212+s.In[9,12]*OMp312
    Fs111 = -s.frc[1,11]+s.m[11]*(ALPHA111+BETA211*s.l[2,11])
    Fs211 = -s.frc[2,11]+s.m[11]*(ALPHA211+BS511*s.l[2,11])
    Fs311 = -s.frc[3,11]+s.m[11]*(ALPHA311+BETA811*s.l[2,11])
    Fq111 = Fs111+Fs112
    Fq211 = Fs211+Fs212*C12-Fs312*S12
    Fq311 = Fs311+Fs212*S12+Fs312*C12
    Cq111 = -s.trq[1,11]+Cq112+s.In[1,11]*OMp111-s.In[5,11]*OM211*OM311+s.In[9,11]*OM211*OM311+Fs311*s.l[2,11]+ \
 	  s.dpt[2,13]*(Fs212*S12+Fs312*C12)
    Cq211 = -s.trq[2,11]+s.In[1,11]*OM111*OM311+s.In[5,11]*OMp211-s.In[9,11]*OM111*OM311+Cq212*C12-Cq312*S12
    Cq311 = -s.trq[3,11]-s.In[1,11]*OM111*OM211+s.In[5,11]*OM111*OM211+s.In[9,11]*OMp311+Cq212*S12+Cq312*C12-Fs111* \
 	  s.l[2,11]-Fs112*s.dpt[2,13]
    Fs110 = -s.frc[1,10]+s.m[10]*(ALPHA110+BETA210*s.l[2,10])
    Fs210 = -s.frc[2,10]+s.m[10]*(ALPHA210+BS510*s.l[2,10])
    Fs310 = -s.frc[3,10]+s.m[10]*(ALPHA310+BETA810*s.l[2,10])
    Cq110 = -s.trq[1,10]+s.In[1,10]*OMp110-s.In[5,10]*OM210*OM310+s.In[9,10]*OM210*OM310+Fs310*s.l[2,10]
    Cq210 = -s.trq[2,10]+s.In[1,10]*OM110*OM310+s.In[5,10]*OMp210-s.In[9,10]*OM110*OM310
    Cq310 = -s.trq[3,10]-s.In[1,10]*OM110*OM210+s.In[5,10]*OM110*OM210+s.In[9,10]*OMp310-Fs110*s.l[2,10]
    Fs19 = -s.frc[1,9]+s.m[9]*(ALPHA19+BETA39*s.l[3,9])
    Fs29 = -s.frc[2,9]+s.m[9]*(ALPHA29+BETA69*s.l[3,9])
    Fs39 = -s.frc[3,9]+s.m[9]*(ALPHA39+BS99*s.l[3,9])
    Cq19 = -s.trq[1,9]+s.In[1,9]*OMp19-s.In[5,9]*OM29*OM39+s.In[9,9]*OM29*OM39-Fs29*s.l[3,9]
    Cq29 = -s.trq[2,9]+s.In[1,9]*OM19*OM39+s.In[5,9]*OMp29-s.In[9,9]*OM19*OM39+Fs19*s.l[3,9]
    Cq39 = -s.trq[3,9]-s.In[1,9]*OM19*OM29+s.In[5,9]*OM19*OM29+s.In[9,9]*OMp39
    Fs18 = -s.frc[1,8]+s.m[8]*(ALPHA18+BETA28*s.l[2,8])
    Fs28 = -s.frc[2,8]+s.m[8]*(ALPHA28+BS58*s.l[2,8])
    Fs38 = -s.frc[3,8]+s.m[8]*(ALPHA38+BETA88*s.l[2,8])
    Fq18 = Fs18+Fs19
    Fq28 = Fs28+Fs29*C9-Fs39*S9
    Fq38 = Fs38+Fs29*S9+Fs39*C9
    Cq18 = -s.trq[1,8]+Cq19+s.In[1,8]*OMp18-s.In[5,8]*OM28*OM38+s.In[9,8]*OM28*OM38+Fs38*s.l[2,8]+s.dpt[2,9]*(Fs29*S9 \
 	  +Fs39*C9)
    Cq28 = -s.trq[2,8]+s.In[1,8]*OM18*OM38+s.In[5,8]*OMp28-s.In[9,8]*OM18*OM38+Cq29*C9-Cq39*S9
    Cq38 = -s.trq[3,8]-s.In[1,8]*OM18*OM28+s.In[5,8]*OM18*OM28+s.In[9,8]*OMp38+Cq29*S9+Cq39*C9-Fs18*s.l[2,8]-Fs19* \
 	  s.dpt[2,9]
    Fs17 = -s.frc[1,7]+s.m[7]*(ALPHA17+BETA27*s.l[2,7])
    Fs27 = -s.frc[2,7]+s.m[7]*(ALPHA27+BS57*s.l[2,7])
    Fs37 = -s.frc[3,7]+s.m[7]*(ALPHA37+BETA87*s.l[2,7])
    Cq17 = -s.trq[1,7]+s.In[1,7]*OMp17-s.In[5,7]*OM27*OM37+s.In[9,7]*OM27*OM37+Fs37*s.l[2,7]
    Cq27 = -s.trq[2,7]+s.In[1,7]*OM17*OM37+s.In[5,7]*OMp27-s.In[9,7]*OM17*OM37
    Cq37 = -s.trq[3,7]-s.In[1,7]*OM17*OM27+s.In[5,7]*OM17*OM27+s.In[9,7]*OMp37-Fs17*s.l[2,7]
    Fs16 = -s.frc[1,6]+s.m[6]*ALPHA16
    Fs26 = -s.frc[2,6]+s.m[6]*ALPHA26
    Fs36 = -s.frc[3,6]+s.m[6]*ALPHA35
    Fq16 = Fq111+Fq18+Fs110+Fs16+Fs17
    Fq26 = Fs26+Fq211*C11+Fq28*C8-Fq311*S11-Fq38*S8+Fs210*C10+Fs27*C7-Fs310*S10-Fs37*S7
    Fq36 = Fs36+Fq211*S11+Fq28*S8+Fq311*C11+Fq38*C8+Fs210*S10+Fs27*S7+Fs310*C10+Fs37*C7
    Cq16 = -s.trq[1,6]+Cq110+Cq111+Cq17+Cq18+s.In[1,6]*OMp16-s.In[5,6]*OM26*OM36+s.In[9,6]*OM26*OM36+s.dpt[2,1]*(Fs27 \
 	  *S7+Fs37*C7)+s.dpt[2,2]*(Fq28*S8+Fq38*C8)+s.dpt[2,4]*(Fs210*S10+Fs310*C10)+s.dpt[2,5]*(Fq211*S11+Fq311*C11)-s.dpt[3,1] \
 	  *(Fs27*C7-Fs37*S7)-s.dpt[3,2]*(Fq28*C8-Fq38*S8)-s.dpt[3,4]*(Fs210*C10-Fs310*S10)-s.dpt[3,5]*(Fq211*C11-Fq311*S11)
    Cq26 = -s.trq[2,6]+s.In[1,6]*OM16*OM36+s.In[5,6]*OMp26-s.In[9,6]*OM16*OM36+Cq210*C10+Cq211*C11+Cq27*C7+Cq28*C8- \
 	  Cq310*S10-Cq311*S11-Cq37*S7-Cq38*S8+Fq111*s.dpt[3,5]+Fq18*s.dpt[3,2]+Fs110*s.dpt[3,4]+Fs17*s.dpt[3,1]-s.dpt[1,1]*(Fs27 \
 	  *S7+Fs37*C7)-s.dpt[1,2]*(Fq28*S8+Fq38*C8)-s.dpt[1,4]*(Fs210*S10+Fs310*C10)-s.dpt[1,5]*(Fq211*S11+Fq311*C11)
    Cq36 = -s.trq[3,6]-s.In[1,6]*OM16*OM26+s.In[5,6]*OM16*OM26+s.In[9,6]*OMp36+Cq210*S10+Cq211*S11+Cq27*S7+Cq28*S8+ \
 	  Cq310*C10+Cq311*C11+Cq37*C7+Cq38*C8-Fq111*s.dpt[2,5]-Fq18*s.dpt[2,2]-Fs110*s.dpt[2,4]-Fs17*s.dpt[2,1]+s.dpt[1,1]*(Fs27 \
 	  *C7-Fs37*S7)+s.dpt[1,2]*(Fq28*C8-Fq38*S8)+s.dpt[1,4]*(Fs210*C10-Fs310*S10)+s.dpt[1,5]*(Fq211*C11-Fq311*S11)
    Fq15 = Fq16*C6-Fq26*S6
    Fq25 = Fq16*S6+Fq26*C6
    Cq15 = Cq16*C6-Cq26*S6
    Cq25 = Cq16*S6+Cq26*C6
    Fq14 = Fq15*C5+Fq36*S5
    Fq34 = -Fq15*S5+Fq36*C5
    Cq14 = Cq15*C5+Cq36*S5
    Fq23 = Fq25*C4-Fq34*S4
    Fq33 = Fq25*S4+Fq34*C4
 
# Symbolic model output

    Qq[1] = Fq14
    Qq[2] = Fq23
    Qq[3] = Fq33
    Qq[4] = Cq14
    Qq[5] = Cq25
    Qq[6] = Cq36
    Qq[7] = Cq17
    Qq[8] = Cq18
    Qq[9] = Cq19
    Qq[10] = Cq110
    Qq[11] = Cq111
    Qq[12] = Cq112

# Number of continuation lines = 2


