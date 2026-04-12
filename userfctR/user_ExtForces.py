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

    Fx = Fy = Fz = 0.0
    Mx = My = Mz = 0.0
    idpt = mbs_data.xfidpt[ixF] # ID of the point of application of the force in MbsData.dpt
    dxF = mbs_data.dpt[1:, idpt] # Coordinates of the force application point in the BODY-FIXED frame

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
        if ixF == wheel_ids[0] or ixF == wheel_ids[1]:
            K_tire = mbs_data.user_model['FrontTire']['K']  # [N/m] raideur verticale pneu avant
            R_tire = mbs_data.user_model['FrontTire']['R']  # [m] rayon du pneu avant
        else:
            K_tire = mbs_data.user_model['RearTire']['K']  # [N/m] raideur verticale pneu arrière
            R_tire = mbs_data.user_model['RearTire']['R']  # [m] rayon du pneu arrière

        # === Calcul cinématique de la roue ===
        pen = 0 # Pénétration du pneu dans le sol
        pen, rz, angslip, angcamb, slip, Pct, Vmct, Rt_ground, dxF_tgc = tgc_car_kine_wheel(
            PxF, RxF, VxF, OMxF, R_tire
        )

        # Correction d'indice (annonce profs du 4 mars)
        dxF_tgc[0:3] = dxF_tgc[1:4]
        dxF = dxF_tgc[0:3]

        # === Cas 1 : Equilibre  ===
        if mbs_data.process == 2:
            if pen > 0:
                Fz = K_tire * pen  # force normale Fz dans [T]
            else:
                Fz = 0.0
            Fx = Fy = Mx = My = Mz = 0.0

        # === Cas 2 : Equilibre dynamique ===
        elif mbs_data.process == 3:
            if pen > 0:
                T_F = np.zeros(4)  # forces dans repère [T]
                T_M = np.zeros(4)  # moments dans repère [T]
                tgc_bakker_contact(T_F, T_M, angslip, angcamb, slip)
            else:
                T_F = np.zeros(4)
                T_M = np.zeros(4)
            
            # Transformation des forces et moments de [T] vers [I]
            # (en supposant que RxF est la matrice de rotation de [I] vers [T])
            F_inertial = matrix_vector_product(Rt_ground, T_F)
            M_inertial = matrix_vector_product(Rt_ground, T_M)

            Fx = F_inertial[1]  # Force X dans le repère inertiel
            Fy = F_inertial[2]  # Force Y dans le repère inert
            Fz = F_inertial[3]  # Force Z dans le repère inertiel
            Mx = M_inertial[0]  # Moment X dans le repère inertiel
            My = M_inertial[1]  # Moment Y dans le repère inertiel
            Mz = M_inertial[2]  # Moment Z dans le repère inertiel

    Swr = mbs_data.SWr[ixF]
    Swr[1:] = [Fx, Fy, Fz, Mx, My, Mz, dxF[0], dxF[1], dxF[2]]

    return Swr