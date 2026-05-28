import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# Ορισμός παραμέτρων του συστήματος
Cm = 0.3
Kl = 0.005
D = 7e-6

# Συνεχές σύστημα G(s)
num_c = [D]
den_c = [Cm, Kl]
sys_c = signal.TransferFunction(num_c, den_c)

# Ορισμός περιόδων δειγματοληψίας
T_opt = 2
T_mid = 10
T_slow = 15

# Διακριτοποίηση (Μέθοδος Tustin / Bilinear)
sys_d_opt = sys_c.to_discrete(T_opt, method='bilinear')
sys_d_mid = sys_c.to_discrete(T_mid, method='bilinear')
sys_d_slow = sys_c.to_discrete(T_slow, method='bilinear')

# Προσομοίωση Βηματικής Απόκρισης
t_c, y_c = signal.step(sys_c, T=np.linspace(0, 300, 1000))
t_opt, y_opt = signal.dstep(sys_d_opt, n=int(300/T_opt))
t_mid, y_mid = signal.dstep(sys_d_mid, n=int(300/T_mid))
t_slow, y_slow = signal.dstep(sys_d_slow, n=int(300/T_slow))

plt.figure(figsize=(10, 6))
plt.plot(t_c, y_c, 'k-', label='Συνεχές Σύστημα')
plt.step(t_opt, np.squeeze(y_opt), 'b--', where='post', label='Διακριτό T=2s')
plt.step(t_mid, np.squeeze(y_mid), 'g.', where='post', label='Διακριτό T=10s')
plt.step(t_slow, np.squeeze(y_slow), 'r:', where='post', label='Διακριτό T=15s')

plt.title('Επίδραση της Περιόδου Δειγματοληψίας T στην Ακρίβεια του Μοντέλου')
plt.xlabel('Χρόνος (sec)')
plt.ylabel('Πίεση p_m (Pa)')
plt.legend()
plt.grid(True)
plt.show()