import numpy as np
from tgc_py import tgc_car_kine_wheel, tgc_bakker_contact
from MBsysPy import matrix_vector_product

def user_ExtForces(PxF, RxF, VxF, OMxF, AxF, OMPxF, mbs_data, tsim, ixF):
    Fx = Fy = Fz = Mx = My = Mz = 0.0
    idpt = mbs_data.xfidpt[ixF]
    dxF = mbs_data.dpt[1:, idpt]  # valeur par défaut si pas un pneu

    wheel_names = ['F_pneu_av_g', 'F_pneu_av_d', 'F_pneu_ar_g', 'F_pneu_ar_d']
    wheel_ids = [mbs_data.extforce_id.get(n) for n in wheel_names]

    if ixF in wheel_ids:
        K_tire = 200000.0  # [N/m] raideur verticale pneu
        R_tire = 0.295     # [m]   rayon nominal pneu MX-5

        # --- ETAPE 1 : Cinématique du contact ---
        pen, rz, angslip, angcamb, slip, Pct, Vmct, Rt_ground, dxF_tgc = \
            tgc_car_kine_wheel(PxF, RxF, VxF, OMxF, R_tire)

 


        # Correction d'indice (annonce profs du 4 mars)
        dxF_tgc[0:3] = dxF_tgc[1:4]
        dxF = dxF_tgc[0:3]

        # --- ETAPE 2 : Force normale (ressort unilatéral) ---
        Fwhl = np.zeros(4)  # forces dans repère [T]
        Mwhl = np.zeros(4)  # moments dans repère [T]

        if pen > 0:
            Fwhl[3] = K_tire * pen  # force normale Fz dans [T]

            # --- ETAPE 3 : Forces tangentielles Bakker-Pacejka ---
            # Uniquement si le véhicule est en mouvement (slip infini si v=0)
            v_long = abs(VxF[1])
            if v_long > 0.5:  # seuil [m/s] : en dessous, on applique seulement Fz
                tgc_bakker_contact(Fwhl, Mwhl, angslip, angcamb, slip)
                # tgc_bakker_contact remplit Fwhl et Mwhl en place, pas de retour

        # --- ETAPE 4 : Changement de base [T] -> [I] ---
        # Rt_ground = R_{I<-T}, donc matrix_vector_product(Rt_ground, vec_T) = vec_I
        F_inertial = matrix_vector_product(Rt_ground, Fwhl)
        M_inertial = matrix_vector_product(Rt_ground, Mwhl)

        Fx = F_inertial[1]
        Fy = F_inertial[2]
        Fz = F_inertial[3]
        Mx = M_inertial[1]
        My = M_inertial[2]
        Mz = M_inertial[3]

    Swr = mbs_data.SWr[ixF]
    Swr[1:] = [Fx, Fy, Fz, Mx, My, Mz, dxF[0], dxF[1], dxF[2]]
    return Swr