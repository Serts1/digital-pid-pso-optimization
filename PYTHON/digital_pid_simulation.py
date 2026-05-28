import numpy as np
import matplotlib.pyplot as plt

# 1. Μοντέλο Κινητήρα (Παράμετροι)
Cm = 0.3;
Kl = 0.005;
D = 7e-6
T = 2
t_sim = 50
w = 1
N = int(t_sim / T)

# Μνήμη παλαιότερων τιμών PID
u_1 = 0;
u_2 = 0
e_1 = 0;
e_2 = 0
x0 = 0.0

t_total = []
y_total = []
u_total = []

# 2. Προσομοίωση
dt_c = 0.1  # Βήμα προσομοίωσης συνεχούς χρόνου

for k in range(N):
    y_k = x0
    e_k = w - y_k

    # Ψηφιακός PID (εξίσωση διαφορών)
    u_k = 0.66 * u_1 + 0.34 * u_2 + 15685.714 * e_1 - 15171.428 * e_2

    # Προσομοίωση κινητήρα για 1 βήμα Τ (Euler approximation για το συνεχές μέρος)
    t_step = np.arange(0, T + dt_c, dt_c)
    y_step = np.zeros_like(t_step)
    y_step[0] = x0

    for i in range(1, len(t_step)):
        y_step[i] = y_step[i - 1] + dt_c * ((D / Cm) * u_k - (Kl / Cm) * y_step[i - 1])

    t_total.extend(t_step + k * T)
    y_total.extend(y_step)
    u_total.extend([u_k] * len(t_step))

    # Ανανέωση μεταβλητών
    x0 = y_step[-1]
    u_2 = u_1;
    u_1 = u_k
    e_2 = e_1;
    e_1 = e_k

# 3. Γραφικές Παραστάσεις
plt.figure(figsize=(10, 8))
plt.subplot(2, 1, 1)
plt.plot(t_total, y_total, 'b', linewidth=1.5, label='Έξοδος Συστήματος')
plt.axhline(w, color='r', linestyle='--', linewidth=1.5, label='Εντολή w=1')
plt.title('Βηματική Απόκριση Κινητήρα')
plt.xlabel('Χρόνος (s)');
plt.ylabel('Ταχύτητα ω (rad/s)')
plt.legend(loc='lower right');
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(t_total, u_total, 'g', linewidth=1.5)
plt.title('Σήμα Ελέγχου u (Ψηφιακός PID)')
plt.xlabel('Χρόνος (s)');
plt.ylabel('u');
plt.grid(True)

plt.tight_layout()
plt.show()