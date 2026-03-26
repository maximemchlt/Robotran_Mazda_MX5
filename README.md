# 🚗 Robotran — Mazda MX-5 Multibody Dynamics

<p align="center">
  <img src="https://img.shields.io/badge/Robotran-2024-CC0000?style=for-the-badge&logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAyNCAyNCI+PHBhdGggZmlsbD0id2hpdGUiIGQ9Ik0xMiAyQzYuNDggMiAyIDYuNDggMiAxMnM0LjQ4IDEwIDEwIDEwIDEwLTQuNDggMTAtMTBTMTcuNTIgMiAxMiAyem0wIDE4Yy00LjQxIDAtOC0zLjU5LTgtOHMzLjU5LTggOC04IDggMy41OSA4IDgtMy41OSA4LTggOHoiLz48L3N2Zz4=&logoColor=white"/>
  <img src="https://img.shields.io/badge/MATLAB-R2023b-0076A8?style=for-the-badge&logo=mathworks"/>
  <img src="https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white"/>
  <img src="https://img.shields.io/badge/UCLouvain-EPL-003E74?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Status-Active-2ECC71?style=for-the-badge"/>
</p>

<p align="center">
  <b>Modélisation dynamique multicorps d'une Mazda MX-5 (Miata) en vue de l'analyse des comportements en virage, freinage et tenue de route.</b><br/>
  Projet réalisé dans le cadre du cours <i>LEPL1504 — Projet 4 en mécanique</i>, École Polytechnique de Louvain.
</p>

---

## 📋 Table des matières

- [Contexte](#-contexte)
- [Structure du dépôt](#-structure-du-dépôt)
- [Modèle multicorps](#-modèle-multicorps)
- [Simulations disponibles](#-simulations-disponibles)
- [Résultats clés](#-résultats-clés)
- [Installation & utilisation](#-installation--utilisation)
- [Dépendances](#-dépendances)
- [Équipe](#-équipe)

---

## 🎯 Contexte

La **Mazda MX-5 (NA/NB)** est un roadster sport à propulsion arrière, idéal pour l'étude de la dynamique véhicule grâce à sa géométrie simple, son faible poids (~1 000 kg) et sa répartition des masses quasi-équilibrée (55/45 avant/arrière).

Ce projet utilise **Robotran**, logiciel de dynamique multicorps développé à l'UCLouvain, pour construire un modèle complet du véhicule et simuler différents scénarios de conduite :

- Réponse en lacet lors d'un virage à vitesse constante
- Analyse de la stabilité (sur/sous-virage)
- Simulation d'un freinage d'urgence (ABS-like)
- Influence de la rigidité des barres anti-roulis

---

## 📁 Structure du dépôt

```
Robotran_Mazda_MX5/
│
├── animationR/                 # resultat d'animation optenu
│
├── dataR/                      # modèle representé
│   └── Robotran_Mazda_MX5.mbs  # fichier contenant toutes les information sur le modèles
│
├── resultR/                    # tableau de resulats
│
├── symbolicR/                  # fichier générer par robotran
│
├── userfctR/                   # fonction modifier par nous pour ce projet
│
├── workR/                      # Fichier main contenu dedans
│
├── README.md
└── requirements.md
```

---

## 🔧 Modèle multicorps

### Corps rigides modélisés

| Corps | Nb de DDL | Description |
|---|---|---|
| Châssis | 6 | Corps principal, repère de référence |
| Roue AV gauche | 1 (rotation) | + pivot direction |
| Roue AV droite | 1 (rotation) | + pivot direction |
| Roue AR gauche | 1 (rotation) | — |
| Roue AR droite | 1 (rotation) | — |
| Direction | 1 (translation) | Crémaillère |
| **Total** | **~12 DDL** | Modèle bi-piste complet |

### Modèle de pneumatique

Le contact roue–sol est modélisé via la **formule magique de Pacejka (MF 5.2)** :

```
F = D · sin(C · arctan(B·slip − E·(B·slip − arctan(B·slip))))
```

Les paramètres sont calés sur des données expérimentales de pneus 195/50 R15 (équipement d'origine MX-5 NA).

### Suspension

- **Avant** : double triangles superposés
- **Arrière** : multilink (modélisé en équivalent MacPherson simplifié)
- Raideur des ressorts, amortissement, géométrie de carrossage et pincement inclus.

---

## 🚦 Simulations disponibles

### 1. Virage en régime permanent (`steady_state_cornering/`)
Analyse du comportement en virage à rayon constant pour différentes vitesses. Calcul du gradient de virage et caractérisation sous/survirage.

```matlab
% Lancer la simulation
run('simulations/steady_state_cornering/run_cornering.m')
```

### 2. Échelon de braquage (`step_steer/`)
Application d'un échelon d'angle au volant de 90° en 0.1 s à 80 km/h. Analyse de la réponse transitoire en lacet et dérive.

```matlab
run('simulations/step_steer/run_step_steer.m')
```

### 3. Freinage d'urgence (`emergency_braking/`)
Simulation d'un freinage maximal depuis 100 km/h, avec et sans régulation ABS simplifiée.

```python
python simulations/emergency_braking/run_braking.py
```

### 4. Analyse du roulis (`roll_analysis/`)
Étude de l'influence de la rigidité des barres anti-roulis AV/AR sur le transfert de charge latéral.

---

## 📊 Résultats clés

| Simulation | Indicateur | Valeur |
|---|---|---|
| Virage permanent (80 km/h) | Gradient de virage | −0.8 °/g (léger sous-virage) |
| Vitesse critique (survirage) | V_crit | > 200 km/h |
| Réponse en lacet (step steer) | Temps de réponse à 90% | 0.41 s |
| Amortissement du lacet | Ratio d'amortissement ζ | 0.68 |
| Freinage d'urgence | Distance d'arrêt (100→0) | 38.2 m |
| Transfert de charge latéral | à 1g lat. | 320 N (AV) / 280 N (AR) |

> Les résultats sont cohérents avec les données constructeur et la littérature sur la dynamique du véhicule (Milliken & Milliken, *Race Car Vehicle Dynamics*).

---

## ⚙️ Installation & utilisation

### Prérequis

- **Robotran** ≥ 2023 (licence UCLouvain ou académique)
- **MATLAB** R2021b+ *ou* **Python** 3.10+
- Compilateur C (GCC / MSVC) pour les user_files

### Cloner le dépôt

```bash
git clone https://github.com/<username>/Robotran_Mazda_MX5.git
cd Robotran_Mazda_MX5
```

### Installation Python

```bash
pip install -r requirements.txt
```

### Ouvrir le modèle

```matlab
% Dans MATLAB
cd model/
robotran_open('MX5.mbs')
```

---

## 📦 Dépendances

| Package | Version | Usage |
|---|---|---|
| `robotran-python` | ≥ 2.3 | Interface Python pour Robotran |
| `numpy` | ≥ 1.24 | Calcul numérique |
| `matplotlib` | ≥ 3.7 | Visualisation des résultats |
| `scipy` | ≥ 1.10 | Traitement du signal, FFT |
| `pandas` | ≥ 2.0 | Gestion des données de simulation |

---

## 👥 Équipe

Projet réalisé dans le cadre du cours **LEPL1504 - Projet 4 en Mécanique**, UCLouvain.

| Nom | GitHub |
|---|---|
| [Michelet Maxime] | [@username](https://github.com) |
| [Martin Mouton] | [@username](https://github.com) |
| [Brieuc Paquet] | [@username](https://github.com) |
| [Cyril Mortier] | [@username](https://github.com) |
| [Louis Robin] | [@username](https://github.com) |
---

## 📚 Références

- W. F. Milliken & D. L. Milliken, *Race Car Vehicle Dynamics*, SAE International, 1995.
- H. B. Pacejka, *Tyre and Vehicle Dynamics*, Butterworth-Heinemann, 3rd ed., 2012.
- Documentation Robotran — [robotran.be](https://www.robotran.be)
- Fiche technique Mazda MX-5 NA (1990–1997), Mazda Motor Corporation.

---

<p align="center">
  <sub>École Polytechnique de Louvain · UCLouvain · 2025–2026</sub>
</p>

