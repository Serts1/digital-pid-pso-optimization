clc; clear; close all;

%% 1. Μοντέλο Κινητήρα
Cm = 0.3; Kl = 0.005; D = 7e-6;
sys_ss = ss(tf(D, [Cm, Kl]));

%% 2. Παράμετροι
T = 2;
t_sim = 50;
w = 1;
x0 = 0;

% Μνήμη παλαιότερων τιμών PID
u_1 = 0; u_2 = 0;
e_1 = 0; e_2 = 0;

% Πίνακες για τα γραφήματα
t_total = []; y_total = []; u_total = [];

%% 3. Προσομοίωση
for k = 1:(t_sim / T)
    y_k = sys_ss.C * x0;
    e_k = w - y_k;
    
    % Ψηφιακός PID
    u_k = 0.66*u_1 + 0.34*u_2 + 15685.714*e_1 - 15171.428*e_2;
    
    % Προσομοίωση κινητήρα για 1 βήμα Τ
    t_step = 0:0.1:T;
    u_step = u_k * ones(size(t_step));
    [y_step, t_out, x_step] = lsim(sys_ss, u_step, t_step, x0);
    
    % Αποθήκευση αποτελεσμάτων βήματος
    t_total = [t_total; t_step' + (k-1)*T];
    y_total = [y_total; y_step];
    u_total = [u_total; u_step'];
    
    % Ανανέωση μεταβλητών για την επόμενη επανάληψη
    x0 = x_step(end, :)';
    u_2 = u_1; u_1 = u_k;
    e_2 = e_1; e_1 = e_k;
end

%% 4. Γραφικές Παραστάσεις
figure;
subplot(2,1,1);
plot(t_total, y_total, 'b', 'LineWidth', 1.5); hold on;
yline(w, 'r--', 'LineWidth', 1.5);
title('Βηματική Απόκριση Κινητήρα');
xlabel('Χρόνος (s)'); ylabel('Ταχύτητα \omega (rad/s)');
legend('Έξοδος Συστήματος', 'Εντολή w=1', 'Location', 'Southeast'); grid on;

subplot(2,1,2);
plot(t_total, u_total, 'g', 'LineWidth', 1.5);
title('Σήμα Ελέγχου u (Ψηφιακός PID)');
xlabel('Χρόνος (s)'); ylabel('u'); grid on;