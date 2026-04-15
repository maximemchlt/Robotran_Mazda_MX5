# -*- coding: utf-8 -*-
import numpy as np

def user_JointForces(mbs_data, tsim):
    mbs_data.Qq[:] = 0.0
    mode = mbs_data.user_model.get('simulation', 'MRU')
    
    jid_dir = mbs_data.joint_id["T2_barre_direction"]
    
    # 1. On récupère les roues Arrière ET Avant pour un vrai freinage
    wheels_rear = [mbs_data.joint_id["R2_Roue_AR_G"], mbs_data.joint_id["R2_Roue_AR_D"]]
    try:
        wheels_front = [mbs_data.joint_id["R2_Roue_AV_G"], mbs_data.joint_id["R2_Roue_AV_D"]]
    except KeyError:
        wheels_front = [] # Sécurité si les noms dans ton .mbs sont différents

    q_target = 0.0
    torque_rear = 0.0
    torque_front = 0.0


    # --- CAS : VIRAGE (inchangé) ---
    if mode == "virage":
        if 2.0 <= tsim < 4.0:
            q_target = 0.02 * np.sin(np.pi * (tsim - 2.0) / 2.0)
        elif 4.0 <= tsim < 6.0:
            q_target = -0.02 * np.sin(np.pi * (tsim - 4.0) / 2.0)


    # =========================================================
    # SCÉNARIO : ÉVITEMENT / DÉPASSEMENT (Pilote automatique)
    # =========================================================
    if mode == "evitement":
        
        # ---------------------------------------------------------
        # A. LE VOLANT (Ta boucle fermée de direction)
        # ---------------------------------------------------------
        jid_Y = mbs_data.joint_id["T2_chassis"]
        jid_Yaw = mbs_data.joint_id["R3_chassis"]
        
        Y_actuel = mbs_data.q[jid_Y]
        Yaw_actuel = mbs_data.q[jid_Yaw]
        
        # Définition de la trajectoire
        Y_cible = 0.0
        if tsim < 2.0:
            Y_cible = 0.0
        elif 2.0 <= tsim < 3.0:
            Y_cible = 3.0 * (0.5 - 0.5 * np.cos(np.pi * (tsim - 2.0) / 1.0))
        elif 3.0 <= tsim < 6.0:
            Y_cible = 3.0  
        elif 6.0 <= tsim < 7.5:
            Y_cible = 3.0 * (0.5 + 0.5 * np.cos(np.pi * (tsim - 6.0) / 1.0))
        else:
            Y_cible = 0.0
            
        # Cerveau du pilote (Volant)
        erreur_Y = Y_cible - Y_actuel
        erreur_Yaw = 0.0 - Yaw_actuel 
        
        Kp_Y = 0.015  
        Kd_Yaw = 0.1  
        
        raw_q_target = (Kp_Y * erreur_Y) + (Kd_Yaw * erreur_Yaw)
        q_target = np.clip(raw_q_target, -0.025, 0.025)

        # ---------------------------------------------------------
        # B. LES PÉDALES (Indépendant de la direction)
        # ---------------------------------------------------------
        # Paramètres modifiables pour tester tes conducteurs :
        t_debut_frein = 1.5   # L'obstacle apparaît ! Le conducteur freine (le volant tourne à 2.0s)
        t_fin_frein = 3.5     # Le conducteur relâche la pédale
        force_freinage = -400.0 # Force totale du freinage (N.m). Plus c'est négatif, plus ça pile.

        if t_debut_frein <= tsim <= t_fin_frein:
            # On applique les freins (60% avant, 40% arrière pour la stabilité)
            torque_front = force_freinage * 0.60
            torque_rear  = force_freinage * 0.40
        else:
            # Le conducteur a le pied levé (roue libre)
            torque_front = 0.0
            torque_rear  = 0.0


    # --- CAS : ACCÉLÉRATION / FREINAGE PUR ---
    if mode == "acceleration" and tsim > 1.0:
        torque_rear = 600.0  
    elif mode == "freinage" and tsim > 1.0:
        torque_rear = -600.0 

    # 1. APPLICATION DU CONTRÔLE DE DIRECTION
    K_steering, D_steering = 1e6, 1e4
    mbs_data.Qq[jid_dir] = -K_steering * (mbs_data.q[jid_dir] - q_target) - D_steering * mbs_data.qd[jid_dir]

    # 2. APPLICATION DES COUPLES (Moteur ou Freins)
    for jid in wheels_rear:
        mbs_data.Qq[jid] = torque_rear
        
    for jid in wheels_front:
        mbs_data.Qq[jid] = torque_front

    return mbs_data.Qq