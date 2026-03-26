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
#	==> Function: F1 - Recursive Direct Dynamics of tree-like MBS
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos

def dirdyna(M, c, s, tsim):
    q = s.q
    qd = s.qd
 
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
    S13 = sin(q[13])
    C13 = cos(q[13])
    S14 = sin(q[14])
    C14 = cos(q[14])
    S15 = sin(q[15])
    C15 = cos(q[15])
    S16 = sin(q[16])
    C16 = cos(q[16])
    S17 = sin(q[17])
    C17 = cos(q[17])
    S18 = sin(q[18])
    C18 = cos(q[18])
    S8p9 = C8*S9+S8*C9
    C8p9 = C8*C9-S8*S9
    S11p12 = C11*S12+S11*C12
    C11p12 = C11*C12-S11*S12
    S13p14 = C13*S14+S13*C14
    C13p14 = C13*C14-S13*S14
 
# Forward Kinematics

    AF24 = -s.g[3]*S4
    AF34 = -s.g[3]*C4
    OM15 = qd[4]*C5
    OM35 = qd[4]*S5
    OA15 = -qd[4]*qd[5]*S5
    OA35 = qd[4]*qd[5]*C5
    AF15 = -AF34*S5
    AF35 = AF34*C5
    AM15_2 = S4*S5
    AM35_2 = -S4*C5
    AM15_3 = -C4*S5
    AM35_3 = C4*C5
    OM16 = qd[5]*S6+OM15*C6
    OM26 = qd[5]*C6-OM15*S6
    OM36 = qd[6]+OM35
    OA16 = -qd[6]*OM15*S6+C6*(OA15+qd[5]*qd[6])
    OA26 = -qd[6]*OM15*C6-S6*(OA15+qd[5]*qd[6])
    BS16 = -OM26*OM26-OM36*OM36
    BS26 = OM16*OM26
    BS36 = OM16*OM36
    BS56 = -OM16*OM16-OM36*OM36
    BS66 = OM26*OM36
    BS96 = -OM16*OM16-OM26*OM26
    BEF26 = BS26-OA35
    BEF36 = BS36+OA26
    BEF46 = BS26+OA35
    BEF66 = BS66-OA16
    BEF76 = BS36-OA26
    BEF86 = BS66+OA16
    AF16 = AF15*C6+AF24*S6
    AF26 = -AF15*S6+AF24*C6
    AM16_1 = C5*C6
    AM26_1 = -C5*S6
    AM16_2 = AM15_2*C6+C4*S6
    AM26_2 = -AM15_2*S6+C4*C6
    AM16_3 = AM15_3*C6+S4*S6
    AM26_3 = -AM15_3*S6+S4*C6
    OB16_4 = C5*C6
    OB26_4 = -C5*S6
    OM17 = qd[7]+OM16
    OM27 = OM26*C7+OM36*S7
    OM37 = -OM26*S7+OM36*C7
    OA27 = C7*(OA26+qd[7]*OM36)+S7*(OA35-qd[7]*OM26)
    OA37 = C7*(OA35-qd[7]*OM26)-S7*(OA26+qd[7]*OM36)
    BS27 = OM17*OM27
    BS57 = -OM17*OM17-OM37*OM37
    BS67 = OM27*OM37
    BEF27 = BS27-OA37
    BEF87 = BS67+OA16
    AF17 = AF16+BEF26*s.dpt[2,1]+BEF36*s.dpt[3,1]+BS16*s.dpt[1,1]
    AF27 = C7*(AF26+BEF46*s.dpt[1,1]+BEF66*s.dpt[3,1]+BS56*s.dpt[2,1])+S7*(AF35+BEF76*s.dpt[1,1]+BEF86*s.dpt[2,1]+ \
 	  BS96*s.dpt[3,1])
    AF37 = C7*(AF35+BEF76*s.dpt[1,1]+BEF86*s.dpt[2,1]+BS96*s.dpt[3,1])-S7*(AF26+BEF46*s.dpt[1,1]+BEF66*s.dpt[3,1]+ \
 	  BS56*s.dpt[2,1])
    AM27_1 = AM26_1*C7+S5*S7
    AM37_1 = -AM26_1*S7+S5*C7
    AM27_2 = AM26_2*C7+AM35_2*S7
    AM37_2 = -AM26_2*S7+AM35_2*C7
    AM27_3 = AM26_3*C7+AM35_3*S7
    AM37_3 = -AM26_3*S7+AM35_3*C7
    OB27_4 = OB26_4*C7+S5*S7
    OB37_4 = -OB26_4*S7+S5*C7
    AM17_4 = OB26_4*s.dpt[3,1]-s.dpt[2,1]*S5
    AM27_4 = C7*(-OB16_4*s.dpt[3,1]+s.dpt[1,1]*S5)+S7*(OB16_4*s.dpt[2,1]-OB26_4*s.dpt[1,1])
    AM37_4 = C7*(OB16_4*s.dpt[2,1]-OB26_4*s.dpt[1,1])-S7*(-OB16_4*s.dpt[3,1]+s.dpt[1,1]*S5)
    OB27_5 = C6*C7
    OB37_5 = -C6*S7
    AM17_5 = s.dpt[3,1]*C6
    AM27_5 = -s.dpt[3,1]*S6*C7+S7*(-s.dpt[1,1]*C6+s.dpt[2,1]*S6)
    AM37_5 = s.dpt[3,1]*S6*S7+C7*(-s.dpt[1,1]*C6+s.dpt[2,1]*S6)
    AM27_6 = s.dpt[1,1]*C7
    AM37_6 = -s.dpt[1,1]*S7
    OM18 = qd[8]+OM16
    OM28 = OM26*C8+OM36*S8
    OM38 = -OM26*S8+OM36*C8
    OA28 = C8*(OA26+qd[8]*OM36)+S8*(OA35-qd[8]*OM26)
    OA38 = C8*(OA35-qd[8]*OM26)-S8*(OA26+qd[8]*OM36)
    BS28 = OM18*OM28
    BS58 = -OM18*OM18-OM38*OM38
    BS68 = OM28*OM38
    BEF28 = BS28-OA38
    BEF88 = BS68+OA16
    AF18 = AF16+BEF26*s.dpt[2,2]+BEF36*s.dpt[3,2]+BS16*s.dpt[1,2]
    AF28 = C8*(AF26+BEF46*s.dpt[1,2]+BEF66*s.dpt[3,2]+BS56*s.dpt[2,2])+S8*(AF35+BEF76*s.dpt[1,2]+BEF86*s.dpt[2,2]+ \
 	  BS96*s.dpt[3,2])
    AF38 = C8*(AF35+BEF76*s.dpt[1,2]+BEF86*s.dpt[2,2]+BS96*s.dpt[3,2])-S8*(AF26+BEF46*s.dpt[1,2]+BEF66*s.dpt[3,2]+ \
 	  BS56*s.dpt[2,2])
    AM28_1 = AM26_1*C8+S5*S8
    AM38_1 = -AM26_1*S8+S5*C8
    AM28_2 = AM26_2*C8+AM35_2*S8
    AM38_2 = -AM26_2*S8+AM35_2*C8
    AM28_3 = AM26_3*C8+AM35_3*S8
    AM38_3 = -AM26_3*S8+AM35_3*C8
    OB28_4 = OB26_4*C8+S5*S8
    OB38_4 = -OB26_4*S8+S5*C8
    AM18_4 = OB26_4*s.dpt[3,2]-s.dpt[2,2]*S5
    AM28_4 = C8*(-OB16_4*s.dpt[3,2]+s.dpt[1,2]*S5)+S8*(OB16_4*s.dpt[2,2]-OB26_4*s.dpt[1,2])
    AM38_4 = C8*(OB16_4*s.dpt[2,2]-OB26_4*s.dpt[1,2])-S8*(-OB16_4*s.dpt[3,2]+s.dpt[1,2]*S5)
    OB28_5 = C6*C8
    OB38_5 = -C6*S8
    AM18_5 = s.dpt[3,2]*C6
    AM28_5 = -s.dpt[3,2]*S6*C8+S8*(-s.dpt[1,2]*C6+s.dpt[2,2]*S6)
    AM38_5 = s.dpt[3,2]*S6*S8+C8*(-s.dpt[1,2]*C6+s.dpt[2,2]*S6)
    AM28_6 = s.dpt[1,2]*C8
    AM38_6 = -s.dpt[1,2]*S8
    OM19 = qd[9]+OM18
    OM29 = OM28*C9+OM38*S9
    OM39 = -OM28*S9+OM38*C9
    OA29 = C9*(OA28+qd[9]*OM38)+S9*(OA38-qd[9]*OM28)
    OA39 = C9*(OA38-qd[9]*OM28)-S9*(OA28+qd[9]*OM38)
    BS39 = OM19*OM39
    BS69 = OM29*OM39
    BS99 = -OM19*OM19-OM29*OM29
    BEF39 = BS39+OA29
    BEF69 = BS69-OA16
    AF19 = AF18+BEF28*s.dpt[2,11]
    AF29 = C9*(AF28+BS58*s.dpt[2,11])+S9*(AF38+BEF88*s.dpt[2,11])
    AF39 = C9*(AF38+BEF88*s.dpt[2,11])-S9*(AF28+BS58*s.dpt[2,11])
    AM29_1 = AM28_1*C9+AM38_1*S9
    AM39_1 = -AM28_1*S9+AM38_1*C9
    AM29_2 = AM28_2*C9+AM38_2*S9
    AM39_2 = -AM28_2*S9+AM38_2*C9
    AM29_3 = AM28_3*C9+AM38_3*S9
    AM39_3 = -AM28_3*S9+AM38_3*C9
    OB29_4 = OB28_4*C9+OB38_4*S9
    OB39_4 = -OB28_4*S9+OB38_4*C9
    AM19_4 = AM18_4-OB38_4*s.dpt[2,11]
    AM29_4 = AM28_4*C9+S9*(AM38_4+OB16_4*s.dpt[2,11])
    AM39_4 = -AM28_4*S9+C9*(AM38_4+OB16_4*s.dpt[2,11])
    OB29_5 = OB28_5*C9+OB38_5*S9
    OB39_5 = -OB28_5*S9+OB38_5*C9
    AM19_5 = AM18_5-OB38_5*s.dpt[2,11]
    AM29_5 = AM28_5*C9+S9*(AM38_5+s.dpt[2,11]*S6)
    AM39_5 = -AM28_5*S9+C9*(AM38_5+s.dpt[2,11]*S6)
    AM19_6 = -s.dpt[2,2]-s.dpt[2,11]*C8
    AM29_6 = AM28_6*C9+AM38_6*S9
    AM39_6 = -AM28_6*S9+AM38_6*C9
    AM29_8 = s.dpt[2,11]*S9
    AM39_8 = s.dpt[2,11]*C9
    OM110 = qd[10]+OM16
    OM210 = OM26*C10+OM36*S10
    OM310 = -OM26*S10+OM36*C10
    OA210 = C10*(OA26+qd[10]*OM36)+S10*(OA35-qd[10]*OM26)
    OA310 = C10*(OA35-qd[10]*OM26)-S10*(OA26+qd[10]*OM36)
    BS210 = OM110*OM210
    BS510 = -OM110*OM110-OM310*OM310
    BS610 = OM210*OM310
    BEF210 = BS210-OA310
    BEF810 = BS610+OA16
    AF110 = AF16+BEF26*s.dpt[2,4]+BEF36*s.dpt[3,4]+BS16*s.dpt[1,4]
    AF210 = C10*(AF26+BEF46*s.dpt[1,4]+BEF66*s.dpt[3,4]+BS56*s.dpt[2,4])+S10*(AF35+BEF76*s.dpt[1,4]+BEF86*s.dpt[2,4]+ \
 	  BS96*s.dpt[3,4])
    AF310 = C10*(AF35+BEF76*s.dpt[1,4]+BEF86*s.dpt[2,4]+BS96*s.dpt[3,4])-S10*(AF26+BEF46*s.dpt[1,4]+BEF66*s.dpt[3,4]+ \
 	  BS56*s.dpt[2,4])
    AM210_1 = AM26_1*C10+S10*S5
    AM310_1 = -AM26_1*S10+C10*S5
    AM210_2 = AM26_2*C10+AM35_2*S10
    AM310_2 = -AM26_2*S10+AM35_2*C10
    AM210_3 = AM26_3*C10+AM35_3*S10
    AM310_3 = -AM26_3*S10+AM35_3*C10
    OB210_4 = OB26_4*C10+S10*S5
    OB310_4 = -OB26_4*S10+C10*S5
    AM110_4 = OB26_4*s.dpt[3,4]-s.dpt[2,4]*S5
    AM210_4 = C10*(-OB16_4*s.dpt[3,4]+s.dpt[1,4]*S5)+S10*(OB16_4*s.dpt[2,4]-OB26_4*s.dpt[1,4])
    AM310_4 = C10*(OB16_4*s.dpt[2,4]-OB26_4*s.dpt[1,4])-S10*(-OB16_4*s.dpt[3,4]+s.dpt[1,4]*S5)
    OB210_5 = C10*C6
    OB310_5 = -S10*C6
    AM110_5 = s.dpt[3,4]*C6
    AM210_5 = -s.dpt[3,4]*C10*S6+S10*(-s.dpt[1,4]*C6+s.dpt[2,4]*S6)
    AM310_5 = s.dpt[3,4]*S10*S6+C10*(-s.dpt[1,4]*C6+s.dpt[2,4]*S6)
    AM210_6 = s.dpt[1,4]*C10
    AM310_6 = -s.dpt[1,4]*S10
    OM111 = qd[11]+OM16
    OM211 = OM26*C11+OM36*S11
    OM311 = -OM26*S11+OM36*C11
    OA211 = C11*(OA26+qd[11]*OM36)+S11*(OA35-qd[11]*OM26)
    OA311 = C11*(OA35-qd[11]*OM26)-S11*(OA26+qd[11]*OM36)
    BS211 = OM111*OM211
    BS511 = -OM111*OM111-OM311*OM311
    BS611 = OM211*OM311
    BEF211 = BS211-OA311
    BEF811 = BS611+OA16
    AF111 = AF16+BEF26*s.dpt[2,5]+BEF36*s.dpt[3,5]+BS16*s.dpt[1,5]
    AF211 = C11*(AF26+BEF46*s.dpt[1,5]+BEF66*s.dpt[3,5]+BS56*s.dpt[2,5])+S11*(AF35+BEF76*s.dpt[1,5]+BEF86*s.dpt[2,5]+ \
 	  BS96*s.dpt[3,5])
    AF311 = C11*(AF35+BEF76*s.dpt[1,5]+BEF86*s.dpt[2,5]+BS96*s.dpt[3,5])-S11*(AF26+BEF46*s.dpt[1,5]+BEF66*s.dpt[3,5]+ \
 	  BS56*s.dpt[2,5])
    AM211_1 = AM26_1*C11+S11*S5
    AM311_1 = -AM26_1*S11+C11*S5
    AM211_2 = AM26_2*C11+AM35_2*S11
    AM311_2 = -AM26_2*S11+AM35_2*C11
    AM211_3 = AM26_3*C11+AM35_3*S11
    AM311_3 = -AM26_3*S11+AM35_3*C11
    OB211_4 = OB26_4*C11+S11*S5
    OB311_4 = -OB26_4*S11+C11*S5
    AM111_4 = OB26_4*s.dpt[3,5]-s.dpt[2,5]*S5
    AM211_4 = C11*(-OB16_4*s.dpt[3,5]+s.dpt[1,5]*S5)+S11*(OB16_4*s.dpt[2,5]-OB26_4*s.dpt[1,5])
    AM311_4 = C11*(OB16_4*s.dpt[2,5]-OB26_4*s.dpt[1,5])-S11*(-OB16_4*s.dpt[3,5]+s.dpt[1,5]*S5)
    OB211_5 = C11*C6
    OB311_5 = -S11*C6
    AM111_5 = s.dpt[3,5]*C6
    AM211_5 = -s.dpt[3,5]*C11*S6+S11*(-s.dpt[1,5]*C6+s.dpt[2,5]*S6)
    AM311_5 = s.dpt[3,5]*S11*S6+C11*(-s.dpt[1,5]*C6+s.dpt[2,5]*S6)
    AM211_6 = s.dpt[1,5]*C11
    AM311_6 = -s.dpt[1,5]*S11
    OM112 = qd[12]+OM111
    OM212 = OM211*C12+OM311*S12
    OM312 = -OM211*S12+OM311*C12
    OA212 = C12*(OA211+qd[12]*OM311)+S12*(OA311-qd[12]*OM211)
    OA312 = C12*(OA311-qd[12]*OM211)-S12*(OA211+qd[12]*OM311)
    BS312 = OM112*OM312
    BS612 = OM212*OM312
    BS912 = -OM112*OM112-OM212*OM212
    BEF312 = BS312+OA212
    BEF612 = BS612-OA16
    AF112 = AF111+BEF211*s.dpt[2,16]
    AF212 = C12*(AF211+BS511*s.dpt[2,16])+S12*(AF311+BEF811*s.dpt[2,16])
    AF312 = C12*(AF311+BEF811*s.dpt[2,16])-S12*(AF211+BS511*s.dpt[2,16])
    AM212_1 = AM211_1*C12+AM311_1*S12
    AM312_1 = -AM211_1*S12+AM311_1*C12
    AM212_2 = AM211_2*C12+AM311_2*S12
    AM312_2 = -AM211_2*S12+AM311_2*C12
    AM212_3 = AM211_3*C12+AM311_3*S12
    AM312_3 = -AM211_3*S12+AM311_3*C12
    OB212_4 = OB211_4*C12+OB311_4*S12
    OB312_4 = -OB211_4*S12+OB311_4*C12
    AM112_4 = AM111_4-OB311_4*s.dpt[2,16]
    AM212_4 = AM211_4*C12+S12*(AM311_4+OB16_4*s.dpt[2,16])
    AM312_4 = -AM211_4*S12+C12*(AM311_4+OB16_4*s.dpt[2,16])
    OB212_5 = OB211_5*C12+OB311_5*S12
    OB312_5 = -OB211_5*S12+OB311_5*C12
    AM112_5 = AM111_5-OB311_5*s.dpt[2,16]
    AM212_5 = AM211_5*C12+S12*(AM311_5+s.dpt[2,16]*S6)
    AM312_5 = -AM211_5*S12+C12*(AM311_5+s.dpt[2,16]*S6)
    AM112_6 = -s.dpt[2,5]-s.dpt[2,16]*C11
    AM212_6 = AM211_6*C12+AM311_6*S12
    AM312_6 = -AM211_6*S12+AM311_6*C12
    AM212_11 = s.dpt[2,16]*S12
    AM312_11 = s.dpt[2,16]*C12
 
# Backward Dynamics

    FF117 = -s.frc[1,18]*C18-s.frc[3,18]*S18
    FF317 = s.frc[1,18]*S18-s.frc[3,18]*C18
    CF117 = -s.trq[1,18]*C18-s.trq[3,18]*S18
    CF317 = s.trq[1,18]*S18-s.trq[3,18]*C18
    FF115 = -s.frc[1,16]*C16-s.frc[3,16]*S16
    FF315 = s.frc[1,16]*S16-s.frc[3,16]*C16
    CF115 = -s.trq[1,16]*C16-s.trq[3,16]*S16
    CF315 = s.trq[1,16]*S16-s.trq[3,16]*C16
    FF114 = -s.frc[1,14]+FF115
    FF214 = -s.frc[2,14]-s.frc[2,16]*C15-FF315*S15
    FF314 = -s.frc[3,14]-s.frc[2,16]*S15+FF315*C15
    CF114 = -s.trq[1,14]+CF115+s.dpt[2,19]*(-s.frc[2,16]*S15+FF315*C15)
    CF214 = -s.trq[2,14]-s.trq[2,16]*C15-CF315*S15
    CF314 = -s.trq[3,14]-s.trq[2,16]*S15+CF315*C15-FF115*s.dpt[2,19]
    FF113 = -s.frc[1,13]+FF117+FF114*C14+FF314*S14
    FF213 = -s.frc[2,13]+FF214-s.frc[2,18]*C17-FF317*S17
    FF313 = -s.frc[3,13]-s.frc[2,18]*S17-FF114*S14+FF314*C14+FF317*C17
    CF113 = -s.trq[1,13]+CF117+CF114*C14+CF314*S14+s.dpt[2,18]*(-s.frc[2,18]*S17+FF317*C17)
    CF213 = -s.trq[2,13]+CF214-s.trq[2,18]*C17-CF317*S17
    CF313 = -s.trq[3,13]-s.trq[2,18]*S17-CF114*S14+CF314*C14+CF317*C17-FF117*s.dpt[2,18]
    FA112 = -s.frc[1,12]+s.m[12]*(AF112+BEF312*s.l[3,12])
    FA212 = -s.frc[2,12]+s.m[12]*(AF212+BEF612*s.l[3,12])
    FA312 = -s.frc[3,12]+s.m[12]*(AF312+BS912*s.l[3,12])
    CF112 = -s.trq[1,12]+s.In[1,12]*OA16-s.In[5,12]*OM212*OM312+s.In[9,12]*OM212*OM312-FA212*s.l[3,12]
    CF212 = -s.trq[2,12]+s.In[1,12]*OM112*OM312+s.In[5,12]*OA212-s.In[9,12]*OM112*OM312+FA112*s.l[3,12]
    CF312 = -s.trq[3,12]-s.In[1,12]*OM112*OM212+s.In[5,12]*OM112*OM212+s.In[9,12]*OA312
    FB112_1 = s.m[12]*AM16_1
    FB212_1 = s.m[12]*AM212_1
    FB312_1 = s.m[12]*AM312_1
    CM112_1 = -FB212_1*s.l[3,12]
    CM212_1 = FB112_1*s.l[3,12]
    FB112_2 = s.m[12]*AM16_2
    FB212_2 = s.m[12]*AM212_2
    FB312_2 = s.m[12]*AM312_2
    CM112_2 = -FB212_2*s.l[3,12]
    CM212_2 = FB112_2*s.l[3,12]
    FB112_3 = s.m[12]*AM16_3
    FB212_3 = s.m[12]*AM212_3
    FB312_3 = s.m[12]*AM312_3
    CM112_3 = -FB212_3*s.l[3,12]
    CM212_3 = FB112_3*s.l[3,12]
    FB112_4 = s.m[12]*(AM112_4+OB212_4*s.l[3,12])
    FB212_4 = s.m[12]*(AM212_4-OB16_4*s.l[3,12])
    FB312_4 = s.m[12]*AM312_4
    CM112_4 = s.In[1,12]*OB16_4-FB212_4*s.l[3,12]
    CM212_4 = s.In[5,12]*OB212_4+FB112_4*s.l[3,12]
    CM312_4 = s.In[9,12]*OB312_4
    FB112_5 = s.m[12]*(AM112_5+OB212_5*s.l[3,12])
    FB212_5 = s.m[12]*(AM212_5-s.l[3,12]*S6)
    FB312_5 = s.m[12]*AM312_5
    CM112_5 = s.In[1,12]*S6-FB212_5*s.l[3,12]
    CM212_5 = s.In[5,12]*OB212_5+FB112_5*s.l[3,12]
    CM312_5 = s.In[9,12]*OB312_5
    FB112_6 = s.m[12]*(AM112_6+s.l[3,12]*S11p12)
    FB212_6 = s.m[12]*AM212_6
    FB312_6 = s.m[12]*AM312_6
    CM112_6 = -FB212_6*s.l[3,12]
    CM212_6 = s.In[5,12]*S11p12+FB112_6*s.l[3,12]
    CM312_6 = s.In[9,12]*C11p12
    FB212_11 = s.m[12]*(AM212_11-s.l[3,12])
    FB312_11 = s.m[12]*AM312_11
    CM112_11 = s.In[1,12]-FB212_11*s.l[3,12]
    FB212_12 = -s.m[12]*s.l[3,12]
    CM112_12 = s.In[1,12]-FB212_12*s.l[3,12]
    FA111 = -s.frc[1,11]+s.m[11]*(AF111+BEF211*s.l[2,11])
    FA211 = -s.frc[2,11]+s.m[11]*(AF211+BS511*s.l[2,11])
    FA311 = -s.frc[3,11]+s.m[11]*(AF311+BEF811*s.l[2,11])
    FF111 = FA111+FA112
    FF211 = FA211+FA212*C12-FA312*S12
    FF311 = FA311+FA212*S12+FA312*C12
    CF111 = -s.trq[1,11]+CF112+s.In[1,11]*OA16-s.In[5,11]*OM211*OM311+s.In[9,11]*OM211*OM311+FA311*s.l[2,11]+ \
 	  s.dpt[2,16]*(FA212*S12+FA312*C12)
    CF211 = -s.trq[2,11]+s.In[1,11]*OM111*OM311+s.In[5,11]*OA211-s.In[9,11]*OM111*OM311+CF212*C12-CF312*S12
    CF311 = -s.trq[3,11]-s.In[1,11]*OM111*OM211+s.In[5,11]*OM111*OM211+s.In[9,11]*OA311+CF212*S12+CF312*C12-FA111* \
 	  s.l[2,11]-FA112*s.dpt[2,16]
    FB111_1 = s.m[11]*AM16_1
    FB211_1 = s.m[11]*AM211_1
    FB311_1 = s.m[11]*AM311_1
    FM111_1 = FB111_1+FB112_1
    FM211_1 = FB211_1+FB212_1*C12-FB312_1*S12
    FM311_1 = FB311_1+FB212_1*S12+FB312_1*C12
    CM111_1 = CM112_1+FB311_1*s.l[2,11]+s.dpt[2,16]*(FB212_1*S12+FB312_1*C12)
    CM211_1 = CM212_1*C12
    CM311_1 = CM212_1*S12-FB111_1*s.l[2,11]-FB112_1*s.dpt[2,16]
    FB111_2 = s.m[11]*AM16_2
    FB211_2 = s.m[11]*AM211_2
    FB311_2 = s.m[11]*AM311_2
    FM111_2 = FB111_2+FB112_2
    FM211_2 = FB211_2+FB212_2*C12-FB312_2*S12
    FM311_2 = FB311_2+FB212_2*S12+FB312_2*C12
    CM111_2 = CM112_2+FB311_2*s.l[2,11]+s.dpt[2,16]*(FB212_2*S12+FB312_2*C12)
    CM211_2 = CM212_2*C12
    CM311_2 = CM212_2*S12-FB111_2*s.l[2,11]-FB112_2*s.dpt[2,16]
    FB111_3 = s.m[11]*AM16_3
    FB211_3 = s.m[11]*AM211_3
    FB311_3 = s.m[11]*AM311_3
    FM111_3 = FB111_3+FB112_3
    FM211_3 = FB211_3+FB212_3*C12-FB312_3*S12
    FM311_3 = FB311_3+FB212_3*S12+FB312_3*C12
    CM111_3 = CM112_3+FB311_3*s.l[2,11]+s.dpt[2,16]*(FB212_3*S12+FB312_3*C12)
    CM211_3 = CM212_3*C12
    CM311_3 = CM212_3*S12-FB111_3*s.l[2,11]-FB112_3*s.dpt[2,16]
    FB111_4 = s.m[11]*(AM111_4-OB311_4*s.l[2,11])
    FB211_4 = s.m[11]*AM211_4
    FB311_4 = s.m[11]*(AM311_4+OB16_4*s.l[2,11])
    FM111_4 = FB111_4+FB112_4
    FM211_4 = FB211_4+FB212_4*C12-FB312_4*S12
    FM311_4 = FB311_4+FB212_4*S12+FB312_4*C12
    CM111_4 = CM112_4+s.In[1,11]*OB16_4+FB311_4*s.l[2,11]+s.dpt[2,16]*(FB212_4*S12+FB312_4*C12)
    CM211_4 = s.In[5,11]*OB211_4+CM212_4*C12-CM312_4*S12
    CM311_4 = s.In[9,11]*OB311_4+CM212_4*S12+CM312_4*C12-FB111_4*s.l[2,11]-FB112_4*s.dpt[2,16]
    FB111_5 = s.m[11]*(AM111_5-OB311_5*s.l[2,11])
    FB211_5 = s.m[11]*AM211_5
    FB311_5 = s.m[11]*(AM311_5+s.l[2,11]*S6)
    FM111_5 = FB111_5+FB112_5
    FM211_5 = FB211_5+FB212_5*C12-FB312_5*S12
    FM311_5 = FB311_5+FB212_5*S12+FB312_5*C12
    CM111_5 = CM112_5+s.In[1,11]*S6+FB311_5*s.l[2,11]+s.dpt[2,16]*(FB212_5*S12+FB312_5*C12)
    CM211_5 = s.In[5,11]*OB211_5+CM212_5*C12-CM312_5*S12
    CM311_5 = s.In[9,11]*OB311_5+CM212_5*S12+CM312_5*C12-FB111_5*s.l[2,11]-FB112_5*s.dpt[2,16]
    FB111_6 = s.m[11]*(-s.dpt[2,5]-s.l[2,11]*C11)
    FB211_6 = s.m[11]*AM211_6
    FB311_6 = s.m[11]*AM311_6
    FM111_6 = FB111_6+FB112_6
    FM211_6 = FB211_6+FB212_6*C12-FB312_6*S12
    FM311_6 = FB311_6+FB212_6*S12+FB312_6*C12
    CM111_6 = CM112_6+FB311_6*s.l[2,11]+s.dpt[2,16]*(FB212_6*S12+FB312_6*C12)
    CM211_6 = s.In[5,11]*S11+CM212_6*C12-CM312_6*S12
    CM311_6 = s.In[9,11]*C11+CM212_6*S12+CM312_6*C12-FB111_6*s.l[2,11]-FB112_6*s.dpt[2,16]
    FB311_11 = s.m[11]*s.l[2,11]
    CM111_11 = s.In[1,11]+CM112_11+FB311_11*s.l[2,11]+s.dpt[2,16]*(FB212_11*S12+FB312_11*C12)
    FA110 = -s.frc[1,10]+s.m[10]*(AF110+BEF210*s.l[2,10])
    FA210 = -s.frc[2,10]+s.m[10]*(AF210+BS510*s.l[2,10])
    FA310 = -s.frc[3,10]+s.m[10]*(AF310+BEF810*s.l[2,10])
    CF110 = -s.trq[1,10]+s.In[1,10]*OA16-s.In[5,10]*OM210*OM310+s.In[9,10]*OM210*OM310+FA310*s.l[2,10]
    CF210 = -s.trq[2,10]+s.In[1,10]*OM110*OM310+s.In[5,10]*OA210-s.In[9,10]*OM110*OM310
    CF310 = -s.trq[3,10]-s.In[1,10]*OM110*OM210+s.In[5,10]*OM110*OM210+s.In[9,10]*OA310-FA110*s.l[2,10]
    FB110_1 = s.m[10]*AM16_1
    FB210_1 = s.m[10]*AM210_1
    FB310_1 = s.m[10]*AM310_1
    CM110_1 = FB310_1*s.l[2,10]
    CM310_1 = -FB110_1*s.l[2,10]
    FB110_2 = s.m[10]*AM16_2
    FB210_2 = s.m[10]*AM210_2
    FB310_2 = s.m[10]*AM310_2
    CM110_2 = FB310_2*s.l[2,10]
    CM310_2 = -FB110_2*s.l[2,10]
    FB110_3 = s.m[10]*AM16_3
    FB210_3 = s.m[10]*AM210_3
    FB310_3 = s.m[10]*AM310_3
    CM110_3 = FB310_3*s.l[2,10]
    CM310_3 = -FB110_3*s.l[2,10]
    FB110_4 = s.m[10]*(AM110_4-OB310_4*s.l[2,10])
    FB210_4 = s.m[10]*AM210_4
    FB310_4 = s.m[10]*(AM310_4+OB16_4*s.l[2,10])
    CM110_4 = s.In[1,10]*OB16_4+FB310_4*s.l[2,10]
    CM210_4 = s.In[5,10]*OB210_4
    CM310_4 = s.In[9,10]*OB310_4-FB110_4*s.l[2,10]
    FB110_5 = s.m[10]*(AM110_5-OB310_5*s.l[2,10])
    FB210_5 = s.m[10]*AM210_5
    FB310_5 = s.m[10]*(AM310_5+s.l[2,10]*S6)
    CM110_5 = s.In[1,10]*S6+FB310_5*s.l[2,10]
    CM210_5 = s.In[5,10]*OB210_5
    CM310_5 = s.In[9,10]*OB310_5-FB110_5*s.l[2,10]
    FB110_6 = s.m[10]*(-s.dpt[2,4]-s.l[2,10]*C10)
    FB210_6 = s.m[10]*AM210_6
    FB310_6 = s.m[10]*AM310_6
    CM110_6 = FB310_6*s.l[2,10]
    CM210_6 = s.In[5,10]*S10
    CM310_6 = s.In[9,10]*C10-FB110_6*s.l[2,10]
    FB310_10 = s.m[10]*s.l[2,10]
    CM110_10 = s.In[1,10]+FB310_10*s.l[2,10]
    FA19 = -s.frc[1,9]+s.m[9]*(AF19+BEF39*s.l[3,9])
    FA29 = -s.frc[2,9]+s.m[9]*(AF29+BEF69*s.l[3,9])
    FA39 = -s.frc[3,9]+s.m[9]*(AF39+BS99*s.l[3,9])
    CF19 = -s.trq[1,9]+s.In[1,9]*OA16-s.In[5,9]*OM29*OM39+s.In[9,9]*OM29*OM39-FA29*s.l[3,9]
    CF29 = -s.trq[2,9]+s.In[1,9]*OM19*OM39+s.In[5,9]*OA29-s.In[9,9]*OM19*OM39+FA19*s.l[3,9]
    CF39 = -s.trq[3,9]-s.In[1,9]*OM19*OM29+s.In[5,9]*OM19*OM29+s.In[9,9]*OA39
    FB19_1 = s.m[9]*AM16_1
    FB29_1 = s.m[9]*AM29_1
    FB39_1 = s.m[9]*AM39_1
    CM19_1 = -FB29_1*s.l[3,9]
    CM29_1 = FB19_1*s.l[3,9]
    FB19_2 = s.m[9]*AM16_2
    FB29_2 = s.m[9]*AM29_2
    FB39_2 = s.m[9]*AM39_2
    CM19_2 = -FB29_2*s.l[3,9]
    CM29_2 = FB19_2*s.l[3,9]
    FB19_3 = s.m[9]*AM16_3
    FB29_3 = s.m[9]*AM29_3
    FB39_3 = s.m[9]*AM39_3
    CM19_3 = -FB29_3*s.l[3,9]
    CM29_3 = FB19_3*s.l[3,9]
    FB19_4 = s.m[9]*(AM19_4+OB29_4*s.l[3,9])
    FB29_4 = s.m[9]*(AM29_4-OB16_4*s.l[3,9])
    FB39_4 = s.m[9]*AM39_4
    CM19_4 = s.In[1,9]*OB16_4-FB29_4*s.l[3,9]
    CM29_4 = s.In[5,9]*OB29_4+FB19_4*s.l[3,9]
    CM39_4 = s.In[9,9]*OB39_4
    FB19_5 = s.m[9]*(AM19_5+OB29_5*s.l[3,9])
    FB29_5 = s.m[9]*(AM29_5-s.l[3,9]*S6)
    FB39_5 = s.m[9]*AM39_5
    CM19_5 = s.In[1,9]*S6-FB29_5*s.l[3,9]
    CM29_5 = s.In[5,9]*OB29_5+FB19_5*s.l[3,9]
    CM39_5 = s.In[9,9]*OB39_5
    FB19_6 = s.m[9]*(AM19_6+s.l[3,9]*S8p9)
    FB29_6 = s.m[9]*AM29_6
    FB39_6 = s.m[9]*AM39_6
    CM19_6 = -FB29_6*s.l[3,9]
    CM29_6 = s.In[5,9]*S8p9+FB19_6*s.l[3,9]
    CM39_6 = s.In[9,9]*C8p9
    FB29_8 = s.m[9]*(AM29_8-s.l[3,9])
    FB39_8 = s.m[9]*AM39_8
    CM19_8 = s.In[1,9]-FB29_8*s.l[3,9]
    FB29_9 = -s.m[9]*s.l[3,9]
    CM19_9 = s.In[1,9]-FB29_9*s.l[3,9]
    FA18 = -s.frc[1,8]+s.m[8]*(AF18+BEF28*s.l[2,8])
    FA28 = -s.frc[2,8]+s.m[8]*(AF28+BS58*s.l[2,8])
    FA38 = -s.frc[3,8]+s.m[8]*(AF38+BEF88*s.l[2,8])
    FF18 = FA18+FA19
    FF28 = FA28+FA29*C9-FA39*S9
    FF38 = FA38+FA29*S9+FA39*C9
    CF18 = -s.trq[1,8]+CF19+s.In[1,8]*OA16-s.In[5,8]*OM28*OM38+s.In[9,8]*OM28*OM38+FA38*s.l[2,8]+s.dpt[2,11]*(FA29*S9 \
 	  +FA39*C9)
    CF28 = -s.trq[2,8]+s.In[1,8]*OM18*OM38+s.In[5,8]*OA28-s.In[9,8]*OM18*OM38+CF29*C9-CF39*S9
    CF38 = -s.trq[3,8]-s.In[1,8]*OM18*OM28+s.In[5,8]*OM18*OM28+s.In[9,8]*OA38+CF29*S9+CF39*C9-FA18*s.l[2,8]-FA19* \
 	  s.dpt[2,11]
    FB18_1 = s.m[8]*AM16_1
    FB28_1 = s.m[8]*AM28_1
    FB38_1 = s.m[8]*AM38_1
    FM18_1 = FB18_1+FB19_1
    FM28_1 = FB28_1+FB29_1*C9-FB39_1*S9
    FM38_1 = FB38_1+FB29_1*S9+FB39_1*C9
    CM18_1 = CM19_1+FB38_1*s.l[2,8]+s.dpt[2,11]*(FB29_1*S9+FB39_1*C9)
    CM28_1 = CM29_1*C9
    CM38_1 = CM29_1*S9-FB18_1*s.l[2,8]-FB19_1*s.dpt[2,11]
    FB18_2 = s.m[8]*AM16_2
    FB28_2 = s.m[8]*AM28_2
    FB38_2 = s.m[8]*AM38_2
    FM18_2 = FB18_2+FB19_2
    FM28_2 = FB28_2+FB29_2*C9-FB39_2*S9
    FM38_2 = FB38_2+FB29_2*S9+FB39_2*C9
    CM18_2 = CM19_2+FB38_2*s.l[2,8]+s.dpt[2,11]*(FB29_2*S9+FB39_2*C9)
    CM28_2 = CM29_2*C9
    CM38_2 = CM29_2*S9-FB18_2*s.l[2,8]-FB19_2*s.dpt[2,11]
    FB18_3 = s.m[8]*AM16_3
    FB28_3 = s.m[8]*AM28_3
    FB38_3 = s.m[8]*AM38_3
    FM18_3 = FB18_3+FB19_3
    FM28_3 = FB28_3+FB29_3*C9-FB39_3*S9
    FM38_3 = FB38_3+FB29_3*S9+FB39_3*C9
    CM18_3 = CM19_3+FB38_3*s.l[2,8]+s.dpt[2,11]*(FB29_3*S9+FB39_3*C9)
    CM28_3 = CM29_3*C9
    CM38_3 = CM29_3*S9-FB18_3*s.l[2,8]-FB19_3*s.dpt[2,11]
    FB18_4 = s.m[8]*(AM18_4-OB38_4*s.l[2,8])
    FB28_4 = s.m[8]*AM28_4
    FB38_4 = s.m[8]*(AM38_4+OB16_4*s.l[2,8])
    FM18_4 = FB18_4+FB19_4
    FM28_4 = FB28_4+FB29_4*C9-FB39_4*S9
    FM38_4 = FB38_4+FB29_4*S9+FB39_4*C9
    CM18_4 = CM19_4+s.In[1,8]*OB16_4+FB38_4*s.l[2,8]+s.dpt[2,11]*(FB29_4*S9+FB39_4*C9)
    CM28_4 = s.In[5,8]*OB28_4+CM29_4*C9-CM39_4*S9
    CM38_4 = s.In[9,8]*OB38_4+CM29_4*S9+CM39_4*C9-FB18_4*s.l[2,8]-FB19_4*s.dpt[2,11]
    FB18_5 = s.m[8]*(AM18_5-OB38_5*s.l[2,8])
    FB28_5 = s.m[8]*AM28_5
    FB38_5 = s.m[8]*(AM38_5+s.l[2,8]*S6)
    FM18_5 = FB18_5+FB19_5
    FM28_5 = FB28_5+FB29_5*C9-FB39_5*S9
    FM38_5 = FB38_5+FB29_5*S9+FB39_5*C9
    CM18_5 = CM19_5+s.In[1,8]*S6+FB38_5*s.l[2,8]+s.dpt[2,11]*(FB29_5*S9+FB39_5*C9)
    CM28_5 = s.In[5,8]*OB28_5+CM29_5*C9-CM39_5*S9
    CM38_5 = s.In[9,8]*OB38_5+CM29_5*S9+CM39_5*C9-FB18_5*s.l[2,8]-FB19_5*s.dpt[2,11]
    FB18_6 = s.m[8]*(-s.dpt[2,2]-s.l[2,8]*C8)
    FB28_6 = s.m[8]*AM28_6
    FB38_6 = s.m[8]*AM38_6
    FM18_6 = FB18_6+FB19_6
    FM28_6 = FB28_6+FB29_6*C9-FB39_6*S9
    FM38_6 = FB38_6+FB29_6*S9+FB39_6*C9
    CM18_6 = CM19_6+FB38_6*s.l[2,8]+s.dpt[2,11]*(FB29_6*S9+FB39_6*C9)
    CM28_6 = s.In[5,8]*S8+CM29_6*C9-CM39_6*S9
    CM38_6 = s.In[9,8]*C8+CM29_6*S9+CM39_6*C9-FB18_6*s.l[2,8]-FB19_6*s.dpt[2,11]
    FB38_8 = s.m[8]*s.l[2,8]
    CM18_8 = s.In[1,8]+CM19_8+FB38_8*s.l[2,8]+s.dpt[2,11]*(FB29_8*S9+FB39_8*C9)
    FA17 = -s.frc[1,7]+s.m[7]*(AF17+BEF27*s.l[2,7])
    FA27 = -s.frc[2,7]+s.m[7]*(AF27+BS57*s.l[2,7])
    FA37 = -s.frc[3,7]+s.m[7]*(AF37+BEF87*s.l[2,7])
    CF17 = -s.trq[1,7]+s.In[1,7]*OA16-s.In[5,7]*OM27*OM37+s.In[9,7]*OM27*OM37+FA37*s.l[2,7]
    CF27 = -s.trq[2,7]+s.In[1,7]*OM17*OM37+s.In[5,7]*OA27-s.In[9,7]*OM17*OM37
    CF37 = -s.trq[3,7]-s.In[1,7]*OM17*OM27+s.In[5,7]*OM17*OM27+s.In[9,7]*OA37-FA17*s.l[2,7]
    FB17_1 = s.m[7]*AM16_1
    FB27_1 = s.m[7]*AM27_1
    FB37_1 = s.m[7]*AM37_1
    CM17_1 = FB37_1*s.l[2,7]
    CM37_1 = -FB17_1*s.l[2,7]
    FB17_2 = s.m[7]*AM16_2
    FB27_2 = s.m[7]*AM27_2
    FB37_2 = s.m[7]*AM37_2
    CM17_2 = FB37_2*s.l[2,7]
    CM37_2 = -FB17_2*s.l[2,7]
    FB17_3 = s.m[7]*AM16_3
    FB27_3 = s.m[7]*AM27_3
    FB37_3 = s.m[7]*AM37_3
    CM17_3 = FB37_3*s.l[2,7]
    CM37_3 = -FB17_3*s.l[2,7]
    FB17_4 = s.m[7]*(AM17_4-OB37_4*s.l[2,7])
    FB27_4 = s.m[7]*AM27_4
    FB37_4 = s.m[7]*(AM37_4+OB16_4*s.l[2,7])
    CM17_4 = s.In[1,7]*OB16_4+FB37_4*s.l[2,7]
    CM27_4 = s.In[5,7]*OB27_4
    CM37_4 = s.In[9,7]*OB37_4-FB17_4*s.l[2,7]
    FB17_5 = s.m[7]*(AM17_5-OB37_5*s.l[2,7])
    FB27_5 = s.m[7]*AM27_5
    FB37_5 = s.m[7]*(AM37_5+s.l[2,7]*S6)
    CM17_5 = s.In[1,7]*S6+FB37_5*s.l[2,7]
    CM27_5 = s.In[5,7]*OB27_5
    CM37_5 = s.In[9,7]*OB37_5-FB17_5*s.l[2,7]
    FB17_6 = s.m[7]*(-s.dpt[2,1]-s.l[2,7]*C7)
    FB27_6 = s.m[7]*AM27_6
    FB37_6 = s.m[7]*AM37_6
    CM17_6 = FB37_6*s.l[2,7]
    CM27_6 = s.In[5,7]*S7
    CM37_6 = s.In[9,7]*C7-FB17_6*s.l[2,7]
    FB37_7 = s.m[7]*s.l[2,7]
    CM17_7 = s.In[1,7]+FB37_7*s.l[2,7]
    FA16 = -s.frc[1,6]+s.m[6]*(AF16+BEF36*s.l[3,6])
    FA26 = -s.frc[2,6]+s.m[6]*(AF26+BEF66*s.l[3,6])
    FA36 = -s.frc[3,6]+s.m[6]*(AF35+BS96*s.l[3,6])
    FF16 = FA110+FA16+FA17+FF111+FF18+FF113*C13+FF313*S13
    FF26 = FA26+FF213+FA210*C10+FA27*C7-FA310*S10-FA37*S7+FF211*C11+FF28*C8-FF311*S11-FF38*S8
    FF36 = FA36+FA210*S10+FA27*S7+FA310*C10+FA37*C7-FF113*S13+FF211*S11+FF28*S8+FF311*C11+FF313*C13+FF38*C8
    CF16 = -s.trq[1,6]+CF110+CF111+CF17+CF18+s.In[1,6]*OA16-s.In[5,6]*OM26*OM36+s.In[9,6]*OM26*OM36+CF113*C13+CF313* \
 	  S13-FA26*s.l[3,6]-FF213*s.dpt[3,7]+s.dpt[2,1]*(FA27*S7+FA37*C7)+s.dpt[2,2]*(FF28*S8+FF38*C8)+s.dpt[2,4]*(FA210*S10+ \
 	  FA310*C10)+s.dpt[2,5]*(FF211*S11+FF311*C11)-s.dpt[3,1]*(FA27*C7-FA37*S7)-s.dpt[3,2]*(FF28*C8-FF38*S8)-s.dpt[3,4]*( \
 	  FA210*C10-FA310*S10)-s.dpt[3,5]*(FF211*C11-FF311*S11)
    CF26 = -s.trq[2,6]+CF213+s.In[1,6]*OM16*OM36+s.In[5,6]*OA26-s.In[9,6]*OM16*OM36+CF210*C10+CF211*C11+CF27*C7+CF28* \
 	  C8-CF310*S10-CF311*S11-CF37*S7-CF38*S8+FA110*s.dpt[3,4]+FA16*s.l[3,6]+FA17*s.dpt[3,1]+FF111*s.dpt[3,5]+FF18*s.dpt[3,2] \
 	  -s.dpt[1,1]*(FA27*S7+FA37*C7)-s.dpt[1,2]*(FF28*S8+FF38*C8)-s.dpt[1,4]*(FA210*S10+FA310*C10)-s.dpt[1,5]*(FF211*S11+ \
 	  FF311*C11)-s.dpt[1,7]*(-FF113*S13+FF313*C13)+s.dpt[3,7]*(FF113*C13+FF313*S13)
    CF36 = -s.trq[3,6]-s.In[1,6]*OM16*OM26+s.In[5,6]*OM16*OM26+s.In[9,6]*OA35-CF113*S13+CF210*S10+CF211*S11+CF27*S7+ \
 	  CF28*S8+CF310*C10+CF311*C11+CF313*C13+CF37*C7+CF38*C8-FA110*s.dpt[2,4]-FA17*s.dpt[2,1]-FF111*s.dpt[2,5]-FF18* \
 	  s.dpt[2,2]+FF213*s.dpt[1,7]+s.dpt[1,1]*(FA27*C7-FA37*S7)+s.dpt[1,2]*(FF28*C8-FF38*S8)+s.dpt[1,4]*(FA210*C10-FA310*S10) \
 	  +s.dpt[1,5]*(FF211*C11-FF311*S11)
    FB16_1 = s.m[6]*AM16_1
    FB26_1 = s.m[6]*AM26_1
    FB36_1 = s.m[6]*S5
    FM16_1 = FB110_1+FB16_1+FB17_1+FM111_1+FM18_1
    FM26_1 = FB26_1+FB210_1*C10+FB27_1*C7-FB310_1*S10-FB37_1*S7+FM211_1*C11+FM28_1*C8-FM311_1*S11-FM38_1*S8
    FM36_1 = FB36_1+FB210_1*S10+FB27_1*S7+FB310_1*C10+FB37_1*C7+FM211_1*S11+FM28_1*S8+FM311_1*C11+FM38_1*C8
    CM16_1 = CM110_1+CM111_1+CM17_1+CM18_1-FB26_1*s.l[3,6]+s.dpt[2,1]*(FB27_1*S7+FB37_1*C7)+s.dpt[2,2]*(FM28_1*S8+ \
 	  FM38_1*C8)+s.dpt[2,4]*(FB210_1*S10+FB310_1*C10)+s.dpt[2,5]*(FM211_1*S11+FM311_1*C11)-s.dpt[3,1]*(FB27_1*C7-FB37_1*S7)- \
 	  s.dpt[3,2]*(FM28_1*C8-FM38_1*S8)-s.dpt[3,4]*(FB210_1*C10-FB310_1*S10)-s.dpt[3,5]*(FM211_1*C11-FM311_1*S11)
    CM26_1 = CM211_1*C11+CM28_1*C8-CM310_1*S10-CM311_1*S11-CM37_1*S7-CM38_1*S8+FB110_1*s.dpt[3,4]+FB16_1*s.l[3,6]+ \
 	  FB17_1*s.dpt[3,1]+FM111_1*s.dpt[3,5]+FM18_1*s.dpt[3,2]-s.dpt[1,1]*(FB27_1*S7+FB37_1*C7)-s.dpt[1,2]*(FM28_1*S8+FM38_1* \
 	  C8)-s.dpt[1,4]*(FB210_1*S10+FB310_1*C10)-s.dpt[1,5]*(FM211_1*S11+FM311_1*C11)
    CM36_1 = CM211_1*S11+CM28_1*S8+CM310_1*C10+CM311_1*C11+CM37_1*C7+CM38_1*C8-FB110_1*s.dpt[2,4]-FB17_1*s.dpt[2,1]- \
 	  FM111_1*s.dpt[2,5]-FM18_1*s.dpt[2,2]+s.dpt[1,1]*(FB27_1*C7-FB37_1*S7)+s.dpt[1,2]*(FM28_1*C8-FM38_1*S8)+s.dpt[1,4]*( \
 	  FB210_1*C10-FB310_1*S10)+s.dpt[1,5]*(FM211_1*C11-FM311_1*S11)
    FB16_2 = s.m[6]*AM16_2
    FB26_2 = s.m[6]*AM26_2
    FB36_2 = s.m[6]*AM35_2
    FM16_2 = FB110_2+FB16_2+FB17_2+FM111_2+FM18_2
    FM26_2 = FB26_2+FB210_2*C10+FB27_2*C7-FB310_2*S10-FB37_2*S7+FM211_2*C11+FM28_2*C8-FM311_2*S11-FM38_2*S8
    FM36_2 = FB36_2+FB210_2*S10+FB27_2*S7+FB310_2*C10+FB37_2*C7+FM211_2*S11+FM28_2*S8+FM311_2*C11+FM38_2*C8
    CM16_2 = CM110_2+CM111_2+CM17_2+CM18_2-FB26_2*s.l[3,6]+s.dpt[2,1]*(FB27_2*S7+FB37_2*C7)+s.dpt[2,2]*(FM28_2*S8+ \
 	  FM38_2*C8)+s.dpt[2,4]*(FB210_2*S10+FB310_2*C10)+s.dpt[2,5]*(FM211_2*S11+FM311_2*C11)-s.dpt[3,1]*(FB27_2*C7-FB37_2*S7)- \
 	  s.dpt[3,2]*(FM28_2*C8-FM38_2*S8)-s.dpt[3,4]*(FB210_2*C10-FB310_2*S10)-s.dpt[3,5]*(FM211_2*C11-FM311_2*S11)
    CM26_2 = CM211_2*C11+CM28_2*C8-CM310_2*S10-CM311_2*S11-CM37_2*S7-CM38_2*S8+FB110_2*s.dpt[3,4]+FB16_2*s.l[3,6]+ \
 	  FB17_2*s.dpt[3,1]+FM111_2*s.dpt[3,5]+FM18_2*s.dpt[3,2]-s.dpt[1,1]*(FB27_2*S7+FB37_2*C7)-s.dpt[1,2]*(FM28_2*S8+FM38_2* \
 	  C8)-s.dpt[1,4]*(FB210_2*S10+FB310_2*C10)-s.dpt[1,5]*(FM211_2*S11+FM311_2*C11)
    CM36_2 = CM211_2*S11+CM28_2*S8+CM310_2*C10+CM311_2*C11+CM37_2*C7+CM38_2*C8-FB110_2*s.dpt[2,4]-FB17_2*s.dpt[2,1]- \
 	  FM111_2*s.dpt[2,5]-FM18_2*s.dpt[2,2]+s.dpt[1,1]*(FB27_2*C7-FB37_2*S7)+s.dpt[1,2]*(FM28_2*C8-FM38_2*S8)+s.dpt[1,4]*( \
 	  FB210_2*C10-FB310_2*S10)+s.dpt[1,5]*(FM211_2*C11-FM311_2*S11)
    FB16_3 = s.m[6]*AM16_3
    FB26_3 = s.m[6]*AM26_3
    FB36_3 = s.m[6]*AM35_3
    FM16_3 = FB110_3+FB16_3+FB17_3+FM111_3+FM18_3
    FM26_3 = FB26_3+FB210_3*C10+FB27_3*C7-FB310_3*S10-FB37_3*S7+FM211_3*C11+FM28_3*C8-FM311_3*S11-FM38_3*S8
    FM36_3 = FB36_3+FB210_3*S10+FB27_3*S7+FB310_3*C10+FB37_3*C7+FM211_3*S11+FM28_3*S8+FM311_3*C11+FM38_3*C8
    CM16_3 = CM110_3+CM111_3+CM17_3+CM18_3-FB26_3*s.l[3,6]+s.dpt[2,1]*(FB27_3*S7+FB37_3*C7)+s.dpt[2,2]*(FM28_3*S8+ \
 	  FM38_3*C8)+s.dpt[2,4]*(FB210_3*S10+FB310_3*C10)+s.dpt[2,5]*(FM211_3*S11+FM311_3*C11)-s.dpt[3,1]*(FB27_3*C7-FB37_3*S7)- \
 	  s.dpt[3,2]*(FM28_3*C8-FM38_3*S8)-s.dpt[3,4]*(FB210_3*C10-FB310_3*S10)-s.dpt[3,5]*(FM211_3*C11-FM311_3*S11)
    CM26_3 = CM211_3*C11+CM28_3*C8-CM310_3*S10-CM311_3*S11-CM37_3*S7-CM38_3*S8+FB110_3*s.dpt[3,4]+FB16_3*s.l[3,6]+ \
 	  FB17_3*s.dpt[3,1]+FM111_3*s.dpt[3,5]+FM18_3*s.dpt[3,2]-s.dpt[1,1]*(FB27_3*S7+FB37_3*C7)-s.dpt[1,2]*(FM28_3*S8+FM38_3* \
 	  C8)-s.dpt[1,4]*(FB210_3*S10+FB310_3*C10)-s.dpt[1,5]*(FM211_3*S11+FM311_3*C11)
    CM36_3 = CM211_3*S11+CM28_3*S8+CM310_3*C10+CM311_3*C11+CM37_3*C7+CM38_3*C8-FB110_3*s.dpt[2,4]-FB17_3*s.dpt[2,1]- \
 	  FM111_3*s.dpt[2,5]-FM18_3*s.dpt[2,2]+s.dpt[1,1]*(FB27_3*C7-FB37_3*S7)+s.dpt[1,2]*(FM28_3*C8-FM38_3*S8)+s.dpt[1,4]*( \
 	  FB210_3*C10-FB310_3*S10)+s.dpt[1,5]*(FM211_3*C11-FM311_3*S11)
    FB16_4 = s.m[6]*OB26_4*s.l[3,6]
    FB26_4 = -s.m[6]*OB16_4*s.l[3,6]
    CM16_4 = CM110_4+CM111_4+CM17_4+CM18_4+s.In[1,6]*OB16_4-FB26_4*s.l[3,6]+s.dpt[2,1]*(FB27_4*S7+FB37_4*C7)+ \
 	  s.dpt[2,2]*(FM28_4*S8+FM38_4*C8)+s.dpt[2,4]*(FB210_4*S10+FB310_4*C10)+s.dpt[2,5]*(FM211_4*S11+FM311_4*C11)-s.dpt[3,1]* \
 	  (FB27_4*C7-FB37_4*S7)-s.dpt[3,2]*(FM28_4*C8-FM38_4*S8)-s.dpt[3,4]*(FB210_4*C10-FB310_4*S10)-s.dpt[3,5]*(FM211_4*C11- \
 	  FM311_4*S11)
    CM26_4 = s.In[5,6]*OB26_4+CM210_4*C10+CM211_4*C11+CM27_4*C7+CM28_4*C8-CM310_4*S10-CM311_4*S11-CM37_4*S7-CM38_4*S8 \
 	  +FB110_4*s.dpt[3,4]+FB16_4*s.l[3,6]+FB17_4*s.dpt[3,1]+FM111_4*s.dpt[3,5]+FM18_4*s.dpt[3,2]-s.dpt[1,1]*(FB27_4*S7+ \
 	  FB37_4*C7)-s.dpt[1,2]*(FM28_4*S8+FM38_4*C8)-s.dpt[1,4]*(FB210_4*S10+FB310_4*C10)-s.dpt[1,5]*(FM211_4*S11+FM311_4*C11)
    CM36_4 = s.In[9,6]*S5+CM210_4*S10+CM211_4*S11+CM27_4*S7+CM28_4*S8+CM310_4*C10+CM311_4*C11+CM37_4*C7+CM38_4*C8- \
 	  FB110_4*s.dpt[2,4]-FB17_4*s.dpt[2,1]-FM111_4*s.dpt[2,5]-FM18_4*s.dpt[2,2]+s.dpt[1,1]*(FB27_4*C7-FB37_4*S7)+s.dpt[1,2]* \
 	  (FM28_4*C8-FM38_4*S8)+s.dpt[1,4]*(FB210_4*C10-FB310_4*S10)+s.dpt[1,5]*(FM211_4*C11-FM311_4*S11)
    FB16_5 = s.m[6]*s.l[3,6]*C6
    FB26_5 = -s.m[6]*s.l[3,6]*S6
    CM16_5 = CM110_5+CM111_5+CM17_5+CM18_5+s.In[1,6]*S6-FB26_5*s.l[3,6]+s.dpt[2,1]*(FB27_5*S7+FB37_5*C7)+s.dpt[2,2]*( \
 	  FM28_5*S8+FM38_5*C8)+s.dpt[2,4]*(FB210_5*S10+FB310_5*C10)+s.dpt[2,5]*(FM211_5*S11+FM311_5*C11)-s.dpt[3,1]*(FB27_5*C7- \
 	  FB37_5*S7)-s.dpt[3,2]*(FM28_5*C8-FM38_5*S8)-s.dpt[3,4]*(FB210_5*C10-FB310_5*S10)-s.dpt[3,5]*(FM211_5*C11-FM311_5*S11)
    CM26_5 = s.In[5,6]*C6+CM210_5*C10+CM211_5*C11+CM27_5*C7+CM28_5*C8-CM310_5*S10-CM311_5*S11-CM37_5*S7-CM38_5*S8+ \
 	  FB110_5*s.dpt[3,4]+FB16_5*s.l[3,6]+FB17_5*s.dpt[3,1]+FM111_5*s.dpt[3,5]+FM18_5*s.dpt[3,2]-s.dpt[1,1]*(FB27_5*S7+FB37_5 \
 	  *C7)-s.dpt[1,2]*(FM28_5*S8+FM38_5*C8)-s.dpt[1,4]*(FB210_5*S10+FB310_5*C10)-s.dpt[1,5]*(FM211_5*S11+FM311_5*C11)
    CM36_5 = CM210_5*S10+CM211_5*S11+CM27_5*S7+CM28_5*S8+CM310_5*C10+CM311_5*C11+CM37_5*C7+CM38_5*C8-FB110_5* \
 	  s.dpt[2,4]-FB17_5*s.dpt[2,1]-FM111_5*s.dpt[2,5]-FM18_5*s.dpt[2,2]+s.dpt[1,1]*(FB27_5*C7-FB37_5*S7)+s.dpt[1,2]*(FM28_5* \
 	  C8-FM38_5*S8)+s.dpt[1,4]*(FB210_5*C10-FB310_5*S10)+s.dpt[1,5]*(FM211_5*C11-FM311_5*S11)
    CM36_6 = s.In[9,6]+CM210_6*S10+CM211_6*S11+CM27_6*S7+CM28_6*S8+CM310_6*C10+CM311_6*C11+CM37_6*C7+CM38_6*C8- \
 	  FB110_6*s.dpt[2,4]-FB17_6*s.dpt[2,1]-FM111_6*s.dpt[2,5]-FM18_6*s.dpt[2,2]+s.dpt[1,1]*(FB27_6*C7-FB37_6*S7)+s.dpt[1,2]* \
 	  (FM28_6*C8-FM38_6*S8)+s.dpt[1,4]*(FB210_6*C10-FB310_6*S10)+s.dpt[1,5]*(FM211_6*C11-FM311_6*S11)
    FF15 = FF16*C6-FF26*S6
    FF25 = FF16*S6+FF26*C6
    CF15 = CF16*C6-CF26*S6
    CF25 = CF16*S6+CF26*C6
    FM15_1 = FM16_1*C6-FM26_1*S6
    FM25_1 = FM16_1*S6+FM26_1*C6
    CM15_1 = CM16_1*C6-CM26_1*S6
    CM25_1 = CM16_1*S6+CM26_1*C6
    FM15_2 = FM16_2*C6-FM26_2*S6
    FM25_2 = FM16_2*S6+FM26_2*C6
    CM15_2 = CM16_2*C6-CM26_2*S6
    CM25_2 = CM16_2*S6+CM26_2*C6
    FM15_3 = FM16_3*C6-FM26_3*S6
    FM25_3 = FM16_3*S6+FM26_3*C6
    CM15_3 = CM16_3*C6-CM26_3*S6
    CM25_3 = CM16_3*S6+CM26_3*C6
    CM15_4 = CM16_4*C6-CM26_4*S6
    CM25_4 = CM16_4*S6+CM26_4*C6
    CM25_5 = CM16_5*S6+CM26_5*C6
    FF14 = FF15*C5+FF36*S5
    FF34 = -FF15*S5+FF36*C5
    CF14 = CF15*C5+CF36*S5
    FM14_1 = FM15_1*C5+FM36_1*S5
    FM34_1 = -FM15_1*S5+FM36_1*C5
    CM14_1 = CM15_1*C5+CM36_1*S5
    FM34_2 = -FM15_2*S5+FM36_2*C5
    CM14_2 = CM15_2*C5+CM36_2*S5
    FM34_3 = -FM15_3*S5+FM36_3*C5
    CM14_3 = CM15_3*C5+CM36_3*S5
    CM14_4 = CM15_4*C5+CM36_4*S5
    FF23 = FF25*C4-FF34*S4
    FF33 = FF25*S4+FF34*C4
    FM23_1 = FM25_1*C4-FM34_1*S4
    FM33_1 = FM25_1*S4+FM34_1*C4
    FM23_2 = FM25_2*C4-FM34_2*S4
    FM33_2 = FM25_2*S4+FM34_2*C4
    FM33_3 = FM25_3*S4+FM34_3*C4
 
# Symbolic model output

    c[1] = FF14
    c[2] = FF23
    c[3] = FF33
    c[4] = CF14
    c[5] = CF25
    c[6] = CF36
    c[7] = CF17
    c[8] = CF18
    c[9] = CF19
    c[10] = CF110
    c[11] = CF111
    c[12] = CF112
    c[13] = CF213
    c[14] = CF214
    c[15] = CF115
    c[16] = -s.trq[2,16]
    c[17] = CF117
    c[18] = -s.trq[2,18]
    M[1,1] = FM14_1
    M[1,2] = FM23_1
    M[1,3] = FM33_1
    M[1,4] = CM14_1
    M[1,5] = CM25_1
    M[1,6] = CM36_1
    M[1,7] = CM17_1
    M[1,8] = CM18_1
    M[1,9] = CM19_1
    M[1,10] = CM110_1
    M[1,11] = CM111_1
    M[1,12] = CM112_1
    M[2,1] = FM23_1
    M[2,2] = FM23_2
    M[2,3] = FM33_2
    M[2,4] = CM14_2
    M[2,5] = CM25_2
    M[2,6] = CM36_2
    M[2,7] = CM17_2
    M[2,8] = CM18_2
    M[2,9] = CM19_2
    M[2,10] = CM110_2
    M[2,11] = CM111_2
    M[2,12] = CM112_2
    M[3,1] = FM33_1
    M[3,2] = FM33_2
    M[3,3] = FM33_3
    M[3,4] = CM14_3
    M[3,5] = CM25_3
    M[3,6] = CM36_3
    M[3,7] = CM17_3
    M[3,8] = CM18_3
    M[3,9] = CM19_3
    M[3,10] = CM110_3
    M[3,11] = CM111_3
    M[3,12] = CM112_3
    M[4,1] = CM14_1
    M[4,2] = CM14_2
    M[4,3] = CM14_3
    M[4,4] = CM14_4
    M[4,5] = CM25_4
    M[4,6] = CM36_4
    M[4,7] = CM17_4
    M[4,8] = CM18_4
    M[4,9] = CM19_4
    M[4,10] = CM110_4
    M[4,11] = CM111_4
    M[4,12] = CM112_4
    M[5,1] = CM25_1
    M[5,2] = CM25_2
    M[5,3] = CM25_3
    M[5,4] = CM25_4
    M[5,5] = CM25_5
    M[5,6] = CM36_5
    M[5,7] = CM17_5
    M[5,8] = CM18_5
    M[5,9] = CM19_5
    M[5,10] = CM110_5
    M[5,11] = CM111_5
    M[5,12] = CM112_5
    M[6,1] = CM36_1
    M[6,2] = CM36_2
    M[6,3] = CM36_3
    M[6,4] = CM36_4
    M[6,5] = CM36_5
    M[6,6] = CM36_6
    M[6,7] = CM17_6
    M[6,8] = CM18_6
    M[6,9] = CM19_6
    M[6,10] = CM110_6
    M[6,11] = CM111_6
    M[6,12] = CM112_6
    M[7,1] = CM17_1
    M[7,2] = CM17_2
    M[7,3] = CM17_3
    M[7,4] = CM17_4
    M[7,5] = CM17_5
    M[7,6] = CM17_6
    M[7,7] = CM17_7
    M[8,1] = CM18_1
    M[8,2] = CM18_2
    M[8,3] = CM18_3
    M[8,4] = CM18_4
    M[8,5] = CM18_5
    M[8,6] = CM18_6
    M[8,8] = CM18_8
    M[8,9] = CM19_8
    M[9,1] = CM19_1
    M[9,2] = CM19_2
    M[9,3] = CM19_3
    M[9,4] = CM19_4
    M[9,5] = CM19_5
    M[9,6] = CM19_6
    M[9,8] = CM19_8
    M[9,9] = CM19_9
    M[10,1] = CM110_1
    M[10,2] = CM110_2
    M[10,3] = CM110_3
    M[10,4] = CM110_4
    M[10,5] = CM110_5
    M[10,6] = CM110_6
    M[10,10] = CM110_10
    M[11,1] = CM111_1
    M[11,2] = CM111_2
    M[11,3] = CM111_3
    M[11,4] = CM111_4
    M[11,5] = CM111_5
    M[11,6] = CM111_6
    M[11,11] = CM111_11
    M[11,12] = CM112_11
    M[12,1] = CM112_1
    M[12,2] = CM112_2
    M[12,3] = CM112_3
    M[12,4] = CM112_4
    M[12,5] = CM112_5
    M[12,6] = CM112_6
    M[12,11] = CM112_11
    M[12,12] = CM112_12

# Number of continuation lines = 3


