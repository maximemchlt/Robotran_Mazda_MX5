# -*- coding: utf-8 -*-
import numpy as np

def user_JointForces(mbs_data, tsim):
    mbs_data.Qq[:] = 0.0
    mode = mbs_data.user_model.get('simulation', 'MRU')
    
    jid_dir = mbs_data.joint_id["T2_barre_direction"]
    wheels_prop = [mbs_data.joint_id["R2_Roue_AR_G"], mbs_data.joint_id["R2_Roue_AR_D"]]

    q_target = 0.0
    torque_drive = 0.0

    # --- CAS : ÉVITEMENT D'OBSTACLE ---
    if mode == "evitement" or mode == "virage":
        if 2.0 <= tsim < 4.0:
            q_target = 0.02 * np.sin(np.pi * (tsim - 2.0) / 2.0)
        elif 4.0 <= tsim < 6.0:
            q_target = -0.02 * np.sin(np.pi * (tsim - 4.0) / 2.0)

    # --- CAS : ACCÉLÉRATION / FREINAGE ---
    if mode == "acceleration" and tsim > 1.0:
        torque_drive = 300.0  
    elif mode == "freinage" and tsim > 1.0:
        torque_drive = -600.0 

    # 1. APPLICATION DU CONTRÔLE DE DIRECTION (Maintien à 0 en MRU)
    K_steering, D_steering = 1e6, 1e4
    mbs_data.Qq[jid_dir] = -K_steering * (mbs_data.q[jid_dir] - q_target) - D_steering * mbs_data.qd[jid_dir]

    # 2. APPLICATION DU COUPLE MOTEUR
    for jid in wheels_prop:
        mbs_data.Qq[jid] = torque_drive

    # =========================================================
    # 3. BARRE ANTI-ROULIS (Le Bug Corrigé)
    # =========================================================
    # Avant (Bras supérieurs)
    jid_av_g, jid_av_d = mbs_data.joint_id["R1_Bras_sup_AV_G"], mbs_data.joint_id["R1_Bras_sup_AV_D"]
    k_bar_av = mbs_data.user_model["FrontSuspension"]["C_bar"]
    # CORRECTION : Addition car les joints sont en miroir !
    diff_av = mbs_data.q[jid_av_g] + mbs_data.q[jid_av_d] 
    # Même signe pour les deux pour transférer la charge correctement
    mbs_data.Qq[jid_av_g] -= k_bar_av * diff_av
    mbs_data.Qq[jid_av_d] -= k_bar_av * diff_av

    # Arrière (Bras inférieurs)
    jid_ar_g, jid_ar_d = mbs_data.joint_id["R1_Bras_inf_AR_G"], mbs_data.joint_id["R1_Bras_inf_AR_D"]
    k_bar_ar = mbs_data.user_model["RearSuspension"]["C_bar"]
    diff_ar = mbs_data.q[jid_ar_g] + mbs_data.q[jid_ar_d]
    mbs_data.Qq[jid_ar_g] -= k_bar_ar * diff_ar
    mbs_data.Qq[jid_ar_d] -= k_bar_ar * diff_ar

    return mbs_data.Qq