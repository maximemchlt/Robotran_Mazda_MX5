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

    if mode == "virage":

        if 2.0 <= tsim < 4.0:

            q_target = 0.02 * np.sin(np.pi * (tsim - 2.0) / 2.0)

        elif 4.0 <= tsim < 6.0:

            q_target = -0.02 * np.sin(np.pi * (tsim - 4.0) / 2.0)


# =========================================================
    # SCÉNARIO : ÉVITEMENT / DÉPASSEMENT (Pilote automatique)
    # =========================================================
    if mode == "evitement" :
        # 1. On récupère les "yeux" du pilote
        jid_Y = mbs_data.joint_id["T2_chassis"]
        jid_Yaw = mbs_data.joint_id["R3_chassis"]
        
        Y_actuel = mbs_data.q[jid_Y]
        Yaw_actuel = mbs_data.q[jid_Yaw]
        
        # 2. Définition de la trajectoire fluide (Courbe en S)
        Y_cible = 0.0
        
        if tsim < 2.0:
            Y_cible = 0.0
            
        elif 2.0 <= tsim < 3.0:
            # Déboîtement : Interpolation douce de 0 à 3m en 1 seconde
            Y_cible = 3.0 * (0.5 - 0.5 * np.cos(np.pi * (tsim - 2.0) / 1.0))
            
        elif 3.0 <= tsim < 6.0:
            Y_cible = 3.0  # Stabilisé sur la bande de gauche
            
        elif 6.0 <= tsim < 7.5:
            # Rabattement : Retour doux de 3m à 0m en 1.5 secondes (on prend plus son temps pour se rabattre)
            Y_cible = 3.0 * (0.5 + 0.5 * np.cos(np.pi * (tsim - 6.0) / 0.5))
            
        else:
            Y_cible = 0.0
        # 3. Le cerveau du pilote (Calcul du coup de volant)
        erreur_Y = Y_cible - Y_actuel
        erreur_Yaw = 0.0 - Yaw_actuel # Le nez doit toujours finir droit (0°)
        
        # Réglage de l'agressivité du pilote
        Kp_Y = 0.015  # Plus c'est grand, plus il jette la voiture vers la cible
        Kd_Yaw = 0.1  # Plus c'est grand, plus il contre-braque fort pour stabiliser
        
        raw_q_target = (Kp_Y * erreur_Y) + (Kd_Yaw * erreur_Yaw)
        
        # Sécurité : On bride les bras du pilote pour ne pas casser la crémaillère
        # (Max +/- 2.5 cm de déplacement sur la barre de direction)
        q_target = np.clip(raw_q_target, -0.025, 0.025)


    # --- CAS : ACCÉLÉRATION / FREINAGE ---
    if mode == "acceleration" and tsim > 1.0:
        torque_drive = 600.0  
    elif mode == "freinage" and tsim > 1.0:
        torque_drive = -600.0 

    # 1. APPLICATION DU CONTRÔLE DE DIRECTION (Maintien à 0 en MRU)
    K_steering, D_steering = 1e6, 1e4
    mbs_data.Qq[jid_dir] = -K_steering * (mbs_data.q[jid_dir] - q_target) - D_steering * mbs_data.qd[jid_dir]

    # 2. APPLICATION DU COUPLE MOTEUR
    for jid in wheels_prop:
        mbs_data.Qq[jid] = torque_drive


    return mbs_data.Qq