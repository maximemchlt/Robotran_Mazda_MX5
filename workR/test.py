#!/usr/bin/env python3
# -*- coding: utf-8 -*-

try:
    import MBsysPy as Robotran
except Exception as e:
    raise ImportError(
        "MBsysPy not found/installed. "
        "See: https://www.robotran.eu/download/how-to-install/"
    ) from e


# =========================================================
# OUTILS DEBUG
# =========================================================
def title(txt):
    print("\n" + "=" * 90)
    print(txt)
    print("=" * 90)

def safe_get(obj, name, default=None):
    try:
        return getattr(obj, name)
    except Exception:
        return default

def vec_to_list(vec):
    """Convertit un vecteur MBsysPy/list/numpy en liste Python lisible."""
    if vec is None:
        return None
    try:
        return list(vec)
    except Exception:
        return vec

def print_vector_with_indices(name, vec, one_based=True, max_len=None):
    title(f"Vecteur : {name}")
    if vec is None:
        print("None")
        return

    try:
        values = list(vec)
    except Exception:
        print(f"Impossible de convertir {name} en liste.")
        print(vec)
        return

    if max_len is not None:
        values = values[:max_len]

    start = 1 if one_based else 0
    for i, v in enumerate(values, start=start):
        print(f"{name}[{i}] = {v}")

def print_object_attrs(obj, obj_name, max_items=300):
    title(f"Attributs disponibles dans {obj_name}")
    names = [n for n in dir(obj) if not n.startswith("__")]
    for i, n in enumerate(names[:max_items], start=1):
        print(f"{i:3d}. {n}")
    if len(names) > max_items:
        print(f"... {len(names) - max_items} attributs supplémentaires non affichés")

def build_joint_reverse_map(mbs_data):
    joint_id = safe_get(mbs_data, "joint_id", {})
    reverse_map = {}
    if isinstance(joint_id, dict):
        for name, idx in joint_id.items():
            reverse_map[idx] = name
    return reverse_map

def print_joint_mapping(mbs_data):
    title("Mapping complet des joints q")
    joint_id = safe_get(mbs_data, "joint_id", None)

    if not isinstance(joint_id, dict):
        print("mbs_data.joint_id indisponible ou pas sous forme de dict.")
        return

    for name, idx in sorted(joint_id.items(), key=lambda kv: kv[1]):
        print(f"q{idx:2d} -> {name}")

def print_q_values(mbs_data):
    title("Valeurs initiales des coordonnées q")
    q = safe_get(mbs_data, "q", None)
    joint_id = safe_get(mbs_data, "joint_id", None)

    if q is None:
        print("mbs_data.q indisponible.")
        return
    if not isinstance(joint_id, dict):
        print("mbs_data.joint_id indisponible.")
        return

    reverse_map = build_joint_reverse_map(mbs_data)

    try:
        q_list = list(q)
    except Exception:
        print("Impossible de convertir mbs_data.q en liste.")
        print(q)
        return

    # MBsysPy est souvent 1-based avec q[0] inutilisé
    if len(q_list) == len(joint_id) + 1:
        for idx in range(1, len(q_list)):
            print(f"q{idx:2d} -> {reverse_map.get(idx, '?'):30s} = {q_list[idx]}")
    else:
        for i, val in enumerate(q_list, start=1):
            print(f"q{i:2d} = {val}")

def print_partition_mapping(mbs_data):
    title("Mapping des coordonnées indépendantes / dépendantes")
    joint_id = safe_get(mbs_data, "joint_id", {})
    reverse_map = build_joint_reverse_map(mbs_data)

    qu = safe_get(mbs_data, "qu", None)
    qv = safe_get(mbs_data, "qv", None)

    if qu is None:
        print("mbs_data.qu indisponible.")
    else:
        try:
            qu_list = list(qu)
            print("\n--- Coordonnées indépendantes : qu = x ---")
            for i, q_idx in enumerate(qu_list, start=1):
                try:
                    q_idx_int = int(q_idx)
                except Exception:
                    q_idx_int = q_idx
                print(f"x{i:2d} = q{q_idx_int:2d} -> {reverse_map.get(q_idx_int, '?')}")
        except Exception:
            print("Impossible de lire mbs_data.qu :", qu)

    if qv is None:
        print("\nmbs_data.qv indisponible.")
    else:
        try:
            qv_list = list(qv)
            print("\n--- Coordonnées dépendantes : qv ---")
            for i, q_idx in enumerate(qv_list, start=1):
                try:
                    q_idx_int = int(q_idx)
                except Exception:
                    q_idx_int = q_idx
                print(f"qv[{i:2d}] = q{q_idx_int:2d} -> {reverse_map.get(q_idx_int, '?')}")
        except Exception:
            print("Impossible de lire mbs_data.qv :", qv)

def print_known_counts(mbs_data):
    title("Compteurs utiles exposés par mbs_data")
    interesting = [
        "njoint", "nqu", "nqv", "nqc", "nbody", "npt", "nloopc",
        "nuserc", "ncons", "nstate"
    ]
    for name in interesting:
        val = safe_get(mbs_data, name, None)
        if val is not None:
            print(f"{name:10s} = {val}")

def try_print_constraints(mbs_data):
    title("Infos contraintes / boucles si exposées")
    for attr in ["lrod_id", "rod_id", "ball_id", "loop_id", "loopc", "cstr_id", "cons_id"]:
        val = safe_get(mbs_data, attr, None)
        if val is not None:
            print(f"\n{attr} = {val}")

def run_equilibrium_debug(mbs_data):
    title("Lancement équilibre avec debug")
    mbs_data.process = 2
    mbs_equil = Robotran.MbsEquil(mbs_data)

    # Important: repartir d'une tolérance propre
    mbs_equil.set_options(method=1, senstol=1e-6, verbose=1)

    print_object_attrs(mbs_equil, "mbs_equil", max_items=150)

    try:
        mbs_equil.run()
        print("\nEquilibrium terminé sans exception Python.")
    except Exception as e:
        print("\nException Python capturée pendant l'équilibre :")
        print(repr(e))

    # Certains attributs peuvent exister après run()
    title("État de mbs_equil après run()")
    for attr in [
        "xs", "xns", "nxs", "iter", "res", "error", "flag", "status",
        "grad", "x", "Rr", "dx"
    ]:
        val = safe_get(mbs_equil, attr, None)
        if val is not None:
            print(f"{attr} = {val}")


# =========================================================
# CHARGEMENT DU PROJET
# =========================================================
mbs_data = Robotran.MbsData("dataR/Robotran_Mazda_MX5.mbs")

um = {}
um['FrontTire']       = {'R': 0.288, 'K': 180000.0}
um['RearTire']        = {'R': 0.288, 'K': 180000.0}

um['FrontSuspension'] = {'K': 27000.0, 'C': 2200.0, 'C_bar': 2500.0, 'Z0': 0.40}
um['RearSuspension']  = {'K': 17000.0, 'C': 1800.0, 'C_bar': 1800.0, 'Z0': 0.40}
mbs_data.user_model = um


# =========================================================
# DEBUG AVANT PARTITIONNEMENT
# =========================================================
title("DEBUG AVANT PARTITIONNEMENT")
print_joint_mapping(mbs_data)
print_q_values(mbs_data)
try_print_constraints(mbs_data)


# =========================================================
# PARTITIONNEMENT
# =========================================================
title("PARTITIONNEMENT")
mbs_data.process = 1
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=1)

print_object_attrs(mbs_part, "mbs_part", max_items=150)

try:
    mbs_part.run()
    print("\nPartitionnement terminé.")
except Exception as e:
    print("\nException Python capturée pendant le partitionnement :")
    print(repr(e))
    raise


# =========================================================
# DEBUG APRES PARTITIONNEMENT
# =========================================================
title("DEBUG APRES PARTITIONNEMENT")
print_known_counts(mbs_data)
print_partition_mapping(mbs_data)

# Affichage brut éventuel
for attr in ["qu", "qv", "qc", "qd", "q"]:
    val = safe_get(mbs_data, attr, None)
    if val is not None:
        print_vector_with_indices(attr, val, one_based=False)


# =========================================================
# EQUILIBRE
# =========================================================
run_equilibrium_debug(mbs_data)

# =========================================================
# DYNAMICS SIMULATION
# =========================================================
title("INITIALISATION DYNAMIQUE")
mbs_data.process = 3
mbs_dyn = Robotran.MbsDyn(mbs_data)
mbs_dyn.set_options(method=1, verbose=1)

print_object_attrs(mbs_dyn, "mbs_dyn", max_items=150)

try:
    mbs_dyn.run()
    print("\nDynamique terminée sans exception Python.")
except Exception as e:
    print("\nException Python capturée pendant la dynamique :")
    print(repr(e))

title("État de mbs_dyn après run()")
for attr in ["t", "q", "qd", "qdd", "Qe", "flag", "status"]:
    val = safe_get(mbs_dyn, attr, None)
    if val is not None:
        print_vector_with_indices(attr, val, one_based=False, max_len=10)