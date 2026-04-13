import numpy as np
from tgc_py import tgc_car_kine_wheel, tgc_bakker_contact
from MBsysPy import matrix_vector_product

def user_ExtForces(PxF, RxF, VxF, OMxF, AxF, OMPxF, mbs_data, tsim, ixF):
    """Calcule les forces extérieures (pneus) pour la Mazda MX-5."""

    Fx = Fy = Fz = 0.0
    Mx = My = Mz = 0.0
    
    # ID du point d'application et coordonnées dans le corps
    idpt = mbs_data.xfidpt[ixF]
    dxF = mbs_data.dpt[1:, idpt]

    # Identification des roues
    wheel_names = ['F_pneu_av_g', 'F_pneu_av_d', 'F_pneu_ar_g', 'F_pneu_ar_d']
    wheel_ids = [mbs_data.extforce_id.get(n) for n in wheel_names]

    if ixF in wheel_ids:
        # Récupération des paramètres selon l'essieu
        if ixF == wheel_ids[0] or ixF == wheel_ids[1]:
            K_tire = mbs_data.user_model['FrontTire']['K']
            R_tire = mbs_data.user_model['FrontTire']['R']
        else:
            K_tire = mbs_data.user_model['RearTire']['K']
            R_tire = mbs_data.user_model['RearTire']['R']

        # === 1. Calcul cinématique de la roue (Pénétration, slip, etc.) ===
        pen, rz, angslip, angcamb, slip, Pct, Vmct, Rt_ground, dxF_tgc = tgc_car_kine_wheel(
            PxF, RxF, VxF, OMxF, R_tire
        )

        # Correction d'indice (Note des profs du 4 mars)
        dxF_tgc[0:3] = dxF_tgc[1:4]
        dxF = dxF_tgc[0:3]

        # === 2. Traitement selon le type de simulation ===
        
        # --- CAS A : ÉQUILIBRE (Process 2) ---
        if mbs_data.process == 2:
            if pen > 0:
                Fz = K_tire * pen  # Modèle linéaire simple pour l'équilibre
            else:
                Fz = 0.0
            Fx = Fy = Mx = My = Mz = 0.0

        # --- CAS B : DYNAMIQUE / MRU (Process 3) ---
        elif mbs_data.process == 3:
            T_F = np.zeros(4)  # Forces dans le repère de contact [T]
            T_M = np.zeros(4)  # Moments dans le repère de contact [T]
            
            if pen > 0:
                # CRITIQUE : Calcul de la charge normale Fz
                Fz_stiffness = K_tire * pen
                
                # SÉCURITÉ : Empêche Fz d'être nul pour éviter les divisions par zéro dans Bakker
                T_F[3] = max(0.1, Fz_stiffness) 

                # ... (Calcul de T_F[3] juste au-dessus)
                
                # APPEL BAKKER : Calcule Fx, Fy et les moments
                tgc_bakker_contact(T_F, T_M, angslip, angcamb, slip)
                
                # === ZONE DE TEST : ON FORCE LES FORCES LATÉRALES À 0 ===
                T_F[2] = 0.0  # Force latérale (celle qui fait zigzaguer)
                T_M[3] = 0.0  # Moment d'alignement (auto-alignant)
                # ========================================================
                
                # Transformation vers le repère inertiel
                F_inertial = matrix_vector_product(Rt_ground, T_F)
                # ...
                M_inertial = matrix_vector_product(Rt_ground, T_M)

                Fx = F_inertial[1]
                Fy = F_inertial[2]
                Fz = F_inertial[3]
                Mx = M_inertial[1]
                My = M_inertial[2]
                Mz = M_inertial[3]
            else:
                # Pas de contact
                Fx = Fy = Fz = Mx = My = Mz = 0.0

    # Remplissage du vecteur de sortie Robotran (Swr)
    # Format : [0.0, Fx, Fy, Fz, Mx, My, Mz, dx, dy, dz]
    Swr = mbs_data.SWr[ixF]
    Swr[1:] = [Fx, Fy, Fz, Mx, My, Mz, dxF[0], dxF[1], dxF[2]]

    return Swr