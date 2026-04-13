# -*- coding: utf-8 -*-
import numpy as np

def user_JointForces(mbs_data, tsim):
    # Nettoyage des forces précédentes
    mbs_data.Qq[:] = 0.0

    # --- 1) VERROUILLAGE DE LA DIRECTION ---
    # On crée une raideur et un amortissement virtuels très élevés pour 
    # forcer la barre de direction à rester à 0.
    jid_dir = mbs_data.joint_id["T2_barre_direction"]
    K_lock = 1e6  # Raideur de maintien
    D_lock = 1e4  # Amortissement de maintien
    mbs_data.Qq[jid_dir] = -K_lock * mbs_data.q[jid_dir] - D_lock * mbs_data.qd[jid_dir]

    # --- 2) BARRE ANTI-ROULIS AVANT ---
    # On agit sur les bras SUPÉRIEURS qui sont les joints indépendants à l'avant
    jid_av_g = mbs_data.joint_id["R1_Bras_sup_AV_G"]
    jid_av_d = mbs_data.joint_id["R1_Bras_sup_AV_D"]
    
    k_bar_av = mbs_data.user_model["FrontSuspension"]["C_bar"]
    diff_av = mbs_data.q[jid_av_g] - mbs_data.q[jid_av_d]
    
    mbs_data.Qq[jid_av_g] -= k_bar_av * diff_av
    mbs_data.Qq[jid_av_d] += k_bar_av * diff_av

    # --- 3) BARRE ANTI-ROULIS ARRIÈRE ---
    # On agit sur les bras INFÉRIEURS qui sont les joints indépendants à l'arrière
    jid_ar_g = mbs_data.joint_id["R1_Bras_inf_AR_G"]
    jid_ar_d = mbs_data.joint_id["R1_Bras_inf_AR_D"]
    
    k_bar_ar = mbs_data.user_model["RearSuspension"]["C_bar"]
    diff_ar = mbs_data.q[jid_ar_g] - mbs_data.q[jid_ar_d]
    
    mbs_data.Qq[jid_ar_g] -= k_bar_ar * diff_ar
    mbs_data.Qq[jid_ar_d] += k_bar_ar * diff_ar

    return mbs_data.Qq