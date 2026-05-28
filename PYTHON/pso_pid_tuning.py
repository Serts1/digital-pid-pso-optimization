import numpy as np
import matplotlib.pyplot as plt

# 1. Παράμετροι Βιομηχανικής Διεργασίας
Cm = 0.3;
Kl = 0.005;
D = 7e-6
T_sim = 60;
dt = 2
N = int(T_sim / dt)
time = np.arange(0, N) * dt

# 2. Σήμα Αναφοράς
w_ref = np.ones(N)

# 3. Παράμετροι Αλγορίθμου PSO
nParticles = 30;
nIter = 20
w_pso = 0.7;
c1 = 1.5;
c2 = 1.5

fp_bounds = [0, 30000]
fi_bounds = [0, 1000]
fd_bounds = [0, 30000]

# 4. Αρχικοποίηση Σωματιδίων
pos = np.zeros((nParticles, 3))
vel = np.zeros((nParticles, 3))

for i in range(nParticles):
    pos[i, 0] = np.random.uniform(fp_bounds[0], fp_bounds[1])
    pos[i, 1] = np.random.uniform(fi_bounds[0], fi_bounds[1])
    pos[i, 2] = np.random.uniform(fd_bounds[0], fd_bounds[1])

pbest = np.copy(pos)
pbest_cost = np.full(nParticles, np.inf)
gbest = np.copy(pos[0, :])
gbest_cost = np.inf

# 5. Κύριος Βρόχος PSO
for iter in range(nIter):
    for i in range(nParticles):
        fp = pos[i, 0]
        fi = pos[i, 1]
        fd = pos[i, 2]

        y = np.zeros(N)
        u = np.zeros(N)
        integral = 0
        prev_error = 0

        for k in range(1, N):
            error = w_ref[k] - y[k - 1]
            integral += error * dt
            derivative = (error - prev_error) / dt
            prev_error = error

            # Υπολογισμός ψηφιακού PID
            u[k] = fp * error + fi * integral + fd * derivative
            y[k] = y[k - 1] + dt * ((D / Cm) * u[k] - (Kl / Cm) * y[k - 1])

        cost = np.sum((w_ref - y) ** 2)

        if cost < pbest_cost[i]:
            pbest_cost[i] = cost
            pbest[i, :] = pos[i, :]

        if cost < gbest_cost:
            gbest_cost = cost
            gbest = np.copy(pos[i, :])

    # Ενημέρωση Ταχύτητας και θέσης Σωματιδίων
    for i in range(nParticles):
        r1 = np.random.rand(3)
        r2 = np.random.rand(3)
        vel[i, :] = w_pso * vel[i, :] + c1 * r1 * (pbest[i, :] - pos[i, :]) + c2 * r2 * (gbest - pos[i, :])
        pos[i, :] = pos[i, :] + vel[i, :]

        # Περιορισμός στα όρια
        pos[i, 0] = np.clip(pos[i, 0], fp_bounds[0], fp_bounds[1])
        pos[i, 1] = np.clip(pos[i, 1], fi_bounds[0], fi_bounds[1])
        pos[i, 2] = np.clip(pos[i, 2], fd_bounds[0], fd_bounds[1])

    print(f'PSO iter {iter + 1:2d} | Best cost = {gbest_cost:.4f}')

# Εκτύπωση αποτελεσμάτων
fp_opt, fi_opt, fd_opt = gbest
print('\n --- Βελτιστοποιημένα Κέρδη PID (PSO) ---')
print(f'fp = {fp_opt:.3f}\nfi = {fi_opt:.3f}\nfd = {fd_opt:.3f}\n')

# 6. Τελική Προσομοίωση με το Βέλτιστο PID
y = np.zeros(N);
u = np.zeros(N)
integral = 0;
prev_error = 0
for k in range(1, N):
    error = w_ref[k] - y[k - 1]
    integral += error * dt
    derivative = (error - prev_error) / dt
    prev_error = error

    u[k] = fp_opt * error + fi_opt * integral + fd_opt * derivative
    y[k] = y[k - 1] + dt * ((D / Cm) * u[k] - (Kl / Cm) * y[k - 1])

# 7. Γραφήματα
plt.figure(figsize=(10, 8))
plt.subplot(2, 1, 1)
plt.plot(time, y, 'b', linewidth=1.5, label='Έξοδος Συστήματος')
plt.plot(time, w_ref, 'r--', linewidth=1.5, label='Σήμα Αναφοράς (w=1)')
plt.ylabel('Ταχύτητα ω (rad/s)')
plt.title('Απόκριση Υδραυλικού Κινητήρα με PSO PID')
plt.legend(loc='lower right');
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(time, u, 'g', linewidth=1.5)
plt.ylabel('Σήμα Ελέγχου (u)');
plt.xlabel('Χρόνος (sec)')
plt.grid(True)

plt.tight_layout()
plt.show()