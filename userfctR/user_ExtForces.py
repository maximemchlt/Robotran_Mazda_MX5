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
    Swr = np.zeros(10)
    Fx = Fy = Fz = 0.0             # forces dans le repère inertiel
    Mx = My = Mz = 0.0             # moments dans le repère inertiel
    idpt = mbs_data.xfidpt[ixF]    # point d'application de la force dans le repère inertiel
    dxF = np.zeros(3)               # position du point d'application de la force dans le repère BODY-FIXED
    dxF[0] = mbs_data.dpt[1, idpt]  # x
    dxF[1] = mbs_data.dpt[2, idpt]  # y
    dxF[2] = mbs_data.dpt[3, idpt]  # z

    back_wheel_names = ['F_pneu_ar_g', 'F_pneu_ar_d']
    
    back_wheel_ids = [mbs_data.extforce_id.get(n) for n in back_wheel_names]

    front_wheel_names = ['F_pneu_av_g', 'F_pneu_av_d']
    
    front_wheel_ids = [mbs_data.extforce_id.get(n) for n in front_wheel_names]

    if ixF in front_wheel_ids or ixF in back_wheel_ids:
        if ixF in front_wheel_ids:
            K_tire = mbs_data.user_model["FrontTire"]["K"]  # [N/m] raideur verticale pneu
            R_tire = mbs_data.user_model["FrontTire"]["R"]  # [m]   rayon nominal pneu MX-5
        else:
            K_tire = mbs_data.user_model["RearTire"]["K"]  # [N/m] raideur verticale pneu
            R_tire = mbs_data.user_model["RearTire"]["R"]     # [m]   rayon nominal pneu MX-5

        # --- ETAPE 1 : Cinématique du contact ---
        pen, rz, angslip, angcamb, slip, Pct, Vmct, Rt_ground, dxF_tgc = tgc_car_kine_wheel(PxF, RxF, VxF, OMxF, R_tire)

        if mbs_data.process == 1:
            Fx = 0.0
            Fy = 0.0
            Fz = mbs_data.user_model["FrontTire"]["K"] * pen
            Mx = 0.0
            My = 0.0
            Mz = 0.0

        # --- Dynamique ---
        elif mbs_data.process == 2:
            if pen > 0.0:
                Fwhl_T[3] = mbs_data.user_model["FrontTire"]["K"] * pen # Force verticale du pneu

                # A adapter à ta fonction Bakker Python existante
                Fwhl_T, Mwhl_T = tgc_bakker_contact(
                    Fwhl_T, Mwhl_T, angslip, angcamb, slip
                )
            else:
                Fwhl_T[:] = 0.0
                Mwhl_T[:] = 0.0
            
        # --- Transformation des forces et moments du repère de contact vers le repère inertiel ---
        Fwhl_I = Rt_ground @ Fwhl_T
        Mwhl_I = Rt_ground @ Mwhl_T
        
    # Remplissage de SWr
    Swr = mbs_data.SWr[ixF]
    Swr[1] = Fwhl_I[1]
    Swr[2] = Fwhl_I[2]
    Swr[3] = Fwhl_I[3]
    Swr[4] = Mwhl_I[1]
    Swr[5] = Mwhl_I[2]
    Swr[6] = Mwhl_I[3]
    Swr[7] = dxF[1]
    Swr[8] = dxF[2]
    Swr[9] = dxF[3]
    
    return Swr
