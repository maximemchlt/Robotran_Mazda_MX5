#!/usr/bin/env python3
import MBsysPy
import os
import numpy as np
import matplotlib.pyplot as plt

print("Starting Mazda MX-5 MBS project!")

mbs_data = MBsysPy.MbsData("dataR/Robotran_Mazda_MX5.mbs")

# =========================================================
# INITIALISATION DU USER MODEL
# =========================================================
um = {}
um['FrontTire']       = {'R': 0.295, 'K': 200000.0}
um['RearTire']        = {'R': 0.295, 'K': 200000.0}
um['FrontSuspension'] = {'K': 27000.0, 'C': 2200.0, 'C_bar': 2500.0, 'Z0': 0.43}
um['RearSuspension']  = {'K': 17000.0, 'C': 1800.0, 'C_bar': 1800.0, 'Z0': 0.43}
mbs_data.user_model = um

# =========================================================
# CONDITIONS INITIALES
# =========================================================
mbs_data.q[3] = 0.118  # roues légèrement en contact avec le sol

# =========================================================
# PARTITIONNEMENT 
# =========================================================
print("\n>> PARTITIONNEMENT...")
mbs_part = MBsysPy.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=1)
mbs_part.run()

# --- ON NE LANCE PLUS MbsEquil ICI CAR IL PLANTE ---

# =========================================================
# 1. SIMULATION FANTÔME (Tassement)
# =========================================================

mbs_data.process = 2
print("\n>> Tassement de la voiture sur place (2 secondes)...")
mbs_dirdyn = MBsysPy.MbsDirdyn(mbs_data)
# save2file=0 : on ne sauvegarde pas, on laisse juste la voiture trouver son équilibre
mbs_dirdyn.set_options(dt0=1e-2, tf=2.0, save2file=1)
mbs_dirdyn.run()

# =========================================================
# 2. INJECTION DU MRU (Voiture stabilisée)
# =========================================================
mbs_data.process = 3
print("\n>> Injection des vitesses (MRU)...")

vitesse_kmh = 70  # [km/h] Vitesse en MRU — mettre 5 pour dos d'âne, 7 pour accélération, 70 pour freinage
vitesse_ms = vitesse_kmh / 3.6  # Modification de la vitesse en mètres par seconde.

mbs_data.qd[1] = vitesse_ms  # On applique vitesse_ms sur le châssis grace à q[1].

# Vitesse angulaire (omega) = Vitesse (vitesse_ms) / Rayon (R)
omega = vitesse_ms / 0.295

# On applique omega aux 4 roues, à la bonne vitesse pour ne pas déraper.
mbs_data.qd[25] = omega  # Roue AV_G
mbs_data.qd[31] = omega  # Roue AV_D
mbs_data.qd[14] = omega  # Roue AR_G
mbs_data.qd[10] = omega  # Roue AR_D

# =========================================================
# 3. LA VRAIE DYNAMIQUE (Enregistrement sur 8s)
# =========================================================
print(f"\n>> LANCEMENT DU MRU ({vitesse_kmh} km/h)...")
# On relance de t=2.0 à t=8.0 avec la sauvegarde activée
mbs_dirdyn.set_options(dt0=1e-2, tf=8.0, save2file=1)
mbs_dirdyn.run()

print("\nSimulation terminée avec succès !")
