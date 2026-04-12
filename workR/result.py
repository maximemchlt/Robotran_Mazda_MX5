# =========================================================
# AFFICHAGE ET SAUVEGARDE DES GRAPHES DE SUSPENSION
# =========================================================
import os
import numpy as np
import matplotlib.pyplot as plt

work_dir = os.path.dirname(os.path.abspath(__file__))
print("\n>> Génération des graphes de suspension...")
results_dir = os.path.normpath(os.path.join(work_dir, "..", "resultsR"))
results_path = os.path.join(results_dir, "dirdyn_q.res")
results = np.loadtxt(results_path)

time = results[:, 0]
q_av_g = results[:, 12]
T3_chassis = results[:, 3]


fig, axs = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
plt.tight_layout(rect=[0, 0.03, 1, 0.95])

"""
# --- BLOC GRAPHE MRU / ACCELERATION / FREINAGE ---
fig.suptitle('Angle des bras de suspension vs Temps - Mazda MX-5', fontsize=16)
plot_save_path = os.path.join(results_dir, "suspension_drop_MRU.png")
"""

# --- BLOC GRAPHE DOS D'ANE ---
fig.suptitle(f'Réponse des Suspensions au Dos d\'âne ( km/h)', fontsize=16)
axs[0, 0].plot(time, q_av_g)
axs[0, 0].set_ylabel('q_av_g')
axs[0, 0].grid(True)

axs[0, 1].plot(time, T3_chassis)
axs[0, 1].set_ylabel('T3_chassis')
axs[0, 1].grid(True)

axs[1, 0].plot(time, q_av_g, label='q_av_g')
axs[1, 0].plot(time, T3_chassis, label='T3_chassis')
axs[1, 0].set_xlabel('Temps (s)')
axs[1, 0].set_ylabel('Valeurs')
axs[1, 0].legend()
axs[1, 0].grid(True)

axs[1, 1].axis('off')

plt.show()

# Plot spring size vs time
spring_size = results[:, 13]  # Adjust column index as needed
fig2, ax = plt.subplots(figsize=(10, 6))
ax.plot(time, spring_size)
ax.set_xlabel('Temps (s)')
ax.set_ylabel('Taille du ressort')
ax.set_title('Taille des ressorts vs Temps')
ax.grid(True)
plt.show()