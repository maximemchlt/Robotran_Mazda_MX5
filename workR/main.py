#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import MBsysPy as Robotran
import os
import numpy as np
import matplotlib.pyplot as plt

# =============================================================================
# 1. PARAMÈTRES DE LA SIMULATION
# =============================================================================
simulation = "freinage"  # Options: "MRU", "acceleration", "freinage", "dos_d_ane", "virage", "evitement"
vitesse_kmh = {"MRU": 36, "acceleration": 7, "freinage": 50, "dos_d_ane": 60, "virage": 50, "evitement": 50}[simulation]

print(f"--- Démarrage du projet Mazda MX-5 : Mode {simulation} ---")

# Chargement du projet
work_dir = os.path.dirname(os.path.abspath(__file__))
mbs_file = os.path.normpath(os.path.join(work_dir, "..", "dataR", "Robotran_Mazda_MX5.mbs"))
mbs_data = Robotran.MbsData(mbs_file)

# =============================================================================
# 2. INITIALISATION DU USER MODEL
# =============================================================================
um = {}
um['simulation']      = simulation
um['FrontTire']       = {'R': 0.288, 'K': 180000.0}
um['RearTire']        = {'R': 0.288, 'K': 180000.0}
um['FrontSuspension'] = {'K': 27000.0, 'C': 2200.0, 'C_bar': 2500.0, 'Z0': 0.43}
um['RearSuspension']  = {'K': 17000.0, 'C': 1800.0, 'C_bar': 1800.0, 'Z0': 0.43}
mbs_data.user_model = um

# Configuration initiale (Hauteur pour garantir le contact pneu/sol)
mbs_data.q[3] = 0.2 

# =============================================================================
# 3. PARTITIONNEMENT
# =============================================================================
print("\n>> PARTITIONNEMENT...")
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=0)
mbs_part.run()



mbs_data.q[39] = mbs_data.qd[39] = mbs_data.qdd[39] = 0




# =============================================================================
# 4. PHASE DE TASSEMENT (Remplace MbsEquil)
# =============================================================================
mbs_data.process = 2
mbs_dirdyn = Robotran.MbsDirdyn(mbs_data)
# On utilise un pas de temps fin (1e-3) pour stabiliser le modèle Bakker
print(">> Phase de tassement (2 secondes)...")
mbs_dirdyn.set_options(dt0=1e-2, tf=2.0, save2file=0) 
mbs_dirdyn.run()

# =============================================================================
# 5. INJECTION DES VITESSES ET SIMULATION DYNAMIQUE
# =============================================================================
print(f">> Injection de la vitesse : {vitesse_kmh} km/h")
mbs_data.process = 3
vitesse_ms = vitesse_kmh / 3.6
omega = vitesse_ms / 0.288 # Vitesse angulaire

# Application des vitesses
mbs_data.qd[1] = vitesse_ms  # Châssis (vitesse longitudinale X)

mbs_data.q[2]  = 0.0  # Y
mbs_data.q[4]  = 0.0  # Roulis (Nouveau !)
mbs_data.q[6]  = 0.0  # Lacet (Yaw)
mbs_data.qd[2] = 0.0  
mbs_data.qd[4] = 0.0  # Vitesse de Roulis (Nouveau !)
mbs_data.qd[6] = 0.0

mbs_data.qd[25] = omega      # Roue AV_G (indices à vérifier selon votre .mbs)
mbs_data.qd[31] = omega      # Roue AV_D
mbs_data.qd[14] = omega      # Roue AR_G
mbs_data.qd[10] = omega      # Roue AR_D
# Application des vitesses
mbs_data.qd[2] = 0.0         # Tuer le glissement latéral parasite (Y)
mbs_data.qd[6] = 0.0         # Tuer la rotation parasite (Yaw)

print(f">> Lancement de la simulation ({simulation})...")
mbs_dirdyn.set_options(dt0=1e-3, tf=10.0, save2file=1)
mbs_dirdyn.run()

# =============================================================================
# 6. GÉNÉRATION DES GRAPHES (PDF)
# =============================================================================
print("\n>> Génération des graphiques...")
results_dir = os.path.normpath(os.path.join(work_dir, "..", "resultsR"))
results_path = os.path.join(results_dir, "dirdyn_q.res")
results = np.loadtxt(results_path)

time = results[:, 0]

# --- MODIFICATION : Récupération dynamique des bonnes colonnes ---
id_av_g = mbs_data.joint_id["R1_Bras_sup_AV_G"] 
id_ar_g = mbs_data.joint_id["R1_Bras_inf_AR_G"] 

q_av_g = results[:, id_av_g] * (180 / np.pi)
q_ar_g = results[:, id_ar_g] * (180 / np.pi)
# -----------------------------------------------------------------

fig, axs = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
# ... (le reste de ton code pour le plot reste identique)
fig.suptitle(f'Réponse des Suspensions - {simulation}', fontsize=12, fontweight='bold')

# Avant Gauche
axs[0].plot(time, q_av_g, color='darkblue', linewidth=1.5, label='Avant Gauche')
axs[0].set_ylabel('Angle (°)')
axs[0].grid(True, linestyle='--', alpha=0.7)
axs[0].legend(loc='best')

# Arrière Gauche
axs[1].plot(time, q_ar_g, color='red', linewidth=1.5, label='Arrière Gauche')
axs[1].set_ylabel('Angle (°)')
axs[1].set_xlabel('Temps (s)')
axs[1].grid(True, linestyle='--', alpha=0.7)
axs[1].legend(loc='best')

plt.tight_layout()

# Sauvegarde PDF pour LaTeX
plot_name = f"suspension_{simulation}.pdf"
if simulation == "dos_d_ane":
    plot_name = f"suspension_dos_d_ane_{vitesse_kmh}kmh.pdf"

plot_save_path = os.path.join(results_dir, plot_name)
plt.savefig(plot_save_path, format="pdf", bbox_inches='tight')
print(f">> Graphique sauvegardé : {plot_save_path}")
plt.show()

print("\n--- Simulation terminée avec succès ---")