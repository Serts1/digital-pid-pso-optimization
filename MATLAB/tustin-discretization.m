clc; clear; close all;

% Ορισμός παραμέτρων του συστήματος
Cm = 0.3;
Kl = 0.005;
D = 7e-6;

% Συνάρτηση Μεταφοράς G(s)
num_c = [D];
den_c = [Cm, Kl];
sys_c = tf(num_c, den_c);

% Ορισμός διαφορετικών περιόδων δειγματοληψίας
T_opt = 2;
T_mid = 10;
T_slow = 15;

% Διακριτοποίηση με τη μέθοδο Tustin (Τραπεζοειδής)
sys_d_opt = c2d(sys_c, T_opt, 'tustin');
sys_d_mid = c2d(sys_c, T_mid, 'tustin');
sys_d_slow = c2d(sys_c, T_slow, 'tustin');

% Προσομοίωση Βηματικής Απόκρισης
figure;
step(sys_c, 'w*', 300); hold on; 
step(sys_d_opt, 'b--', 300);
step(sys_d_mid, 'g.', 300);
step(sys_d_slow, 'r:', 300);

title('Επίδραση της Περιόδου Δειγματοληψίας T στην Ακρίβεια του Μοντέλου');
xlabel('Χρόνος (sec)'); ylabel('Πίεση p_m (Pa)');
legend('Συνεχές Σύστημα', 'Διακριτό T=2s', 'Διακριτό T=10s', 'Διακριτό T=15s');
grid on;