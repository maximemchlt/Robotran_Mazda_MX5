# -*- coding: utf-8 -*-
"""Module for the definition of joint forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


def user_JointForces(mbs_data, tsim):
    """Compute the force and torques in the joint.

    It fills the MBsysPy.MbsData.Qq array.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.

    Notes
    -----
    The numpy.ndarray MBsysPy.MbsData.Qq is 1D array with index starting at 1.
    The first index (array[0]) must not be modified. The first index to be
    filled is array[1].

    Returns
    -------
    None
    """
    # cleaning previous forces value
    mbs_data.Qq[:] = 0.0

    # --- 1) Précharge pour l'équilibre ---
    if tsim == 0.0:
        try:
            mbs_data.Qq[mbs_data.joint_id["R2_Roue_AR_D"]]       = 0.0
            mbs_data.Qq[mbs_data.joint_id["R2_Roue_AR_G"]]       = 0.0
            mbs_data.Qq[mbs_data.joint_id["T2_barre_direction"]] = 0.0
        except Exception:
            pass

    # --- 2) Barre anti-roulis avant et arrière ---
    try:
        jid = mbs_data.joint_id["R2_Roue_AR_D"]
        mbs_data.Qq[jid] = -mbs_data.user_model["FrontSuspension"]["C_bar"] * mbs_data.q[jid]
    except Exception:
        pass

    try:
        jid = mbs_data.joint_id["R2_Roue_AR_G"]
        mbs_data.Qq[jid] = -mbs_data.user_model["RearSuspension"]["C_bar"] * mbs_data.q[jid]
    except Exception:
        pass



    # Example: damping in joint number 5
    # D = 0.5 # N/(m/s)
    # mbs_data.Qq[5] = -D * mbs_data.qd[5]

    return mbs_data.Qq
