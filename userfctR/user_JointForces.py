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


    return mbs_data.Qq