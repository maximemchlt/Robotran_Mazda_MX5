# -*- coding: utf-8 -*-
"""Module for defining user function required to compute external forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2019
import numpy as np
from tgc_py import tgc_car_kine_wheel, tgc_bakker_contact
from MBsysPy import matrix_vector_product


def user_ExtForces(PxF, RxF, VxF, OMxF, AxF, OMPxF, mbs_data, tsim, ixF):
    """Compute an user-specified external force.

    Parameters
    ----------
    PxF : numpy.ndarray
        Position vector (index starting at 1) of the force sensor expressed in
        the inertial frame: PxF[1:4] = [P_x, P_y, P_z]
    RxF : numpy.ndarray
        Rotation matrix (index starting at 1) from the inertial frame to the
        force sensor frame: Frame_sensor = RxF[1:4,1:4] * Frame_inertial
    VxF : numpy.ndarray
        Velocity vector (index starting at 1) of the force sensor expressed in
        the inertial frame: VxF[1:4] = [V_x, V_y, V_z]
    OMxF : numpy.ndarray
        Angular velocity vector (index starting at 1) of the force sensor
        expressed in the inertial frame: OMxF[1:4] = [OM_x, OM_y, OM_z]
    AxF : numpy.ndarray
        Acceleration vector (index starting at 1) of the force sensor expressed
        in the inertial frame: AxF[1:4] = [A_x, A_y, A_z]
    OMPxF : numpy.ndarray
        Angular acceleration vector (index starting at 1) of the force sensor
        expressed in the inertial frame: OMPxF[1:4] = [OMP_x, OMP_y, OMP_z]
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.
    ixF : int
        The ID identifying the computed force sensor.

    Notes
    -----
    For 1D numpy.ndarray with index starting at 1, the first index (array[0])
    must not be modified. The first index to be filled is array[1].

    For 2D numpy.ndarray with index starting at 1, the first row (mat[0, :]) and
    line (mat[:,0]) must not be modified. The subarray to be filled is mat[1:, 1:].

    Returns
    -------
    Swr : numpy.ndarray
        An array of length 10 equal to [0., Fx, Fy, Fz, Mx, My, Mz, dxF].
        F_# are the forces components expressed in inertial frame.
        M_# are the torques components expressed in inertial frame.
        dxF is an array of length 3 containing the component of the forces/torque
        application point expressed in the BODY-FIXED frame.
        This array is a specific line of MbsData.SWr.
    """

    Fx = 0.0
    Fy = 0.0
    Fz = 0.0
    Mx = 0.0
    My = 0.0
    Mz = 0.0
    idpt = mbs_data.xfidpt[ixF]
    dxF = mbs_data.dpt[1:, idpt]

    # Example : Contact force with a wall when X coordinate is higher than 1m.
    #           The force is perfectly horizontal (inertial frame)
    # xlim = 1.0 # m
    # kwall= 1e5 # N/m
    # if PxF[1]>xlim:
    #     Fx = (PxF[1]-xlim)*kwall

    # Concatenating force, torque and force application point to returned array.
    # This must not be modified.

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

        
        # --- DOS D'ANE ---
        # Décommenter ce bloc pour la simulation dos d'âne (et commenter Z0=0.650000 dans LinkForces)
        x_wheel = PxF[1]
        z_road = 0.0
        bosse_start = 16.0   # Position de début de la bosse [m]
        bosse_width = 1     # Largeur de la bosse [m]
        bosse_height = 0.1    # Hauteur de la bosse [m]
        if bosse_start <= x_wheel <= (bosse_start + bosse_width):
            rel_pos = (x_wheel - bosse_start) / bosse_width
            z_road = bosse_height * np.sin(np.pi * rel_pos)**2
        pen += z_road
        

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
