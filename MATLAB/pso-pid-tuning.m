clc; clear; close all;

% 1. Παράμετροι Βιομηχανικής Διεργασίας
Cm = 0.3;
Kl = 0.005;
D = 7e-6;

% Παράμετροι Προσομοίωσης
T_sim = 60;
dt = 2;
N = round(T_sim / dt);
time = (0:N-1) * dt;

% 2. Σήμα Αναφοράς
w_ref = ones(1, N);

% 3. Παράμετροι Αλγορίθμου PSO
nParticles = 30;
nIter = 20;
w_pso = 0.7;
c1 = 1.5;
c2 = 1.5;

% Όρια αναζήτησης κερδών
Kp_bounds = [0, 30000];
Ki_bounds = [0, 1000];
Kd_bounds = [0, 30000];

% 4. Αρχικοποίηση Σωματιδίων
pos = zeros(nParticles, 3);
vel = zeros(nParticles, 3);

for i = 1:nParticles
    pos(i,1) = Kp_bounds(1) + rand() * (Kp_bounds(2) - Kp_bounds(1));
    pos(i,2) = Ki_bounds(1) + rand() * (Ki_bounds(2) - Ki_bounds(1));
    pos(i,3) = Kd_bounds(1) + rand() * (Kd_bounds(2) - Kd_bounds(1));
end

pbest = pos;
pbest_cost = inf(nParticles, 1);
gbest = pos(1, :);
gbest_cost = inf;

% 5. Κύριος Βρόχος PSO
for iter = 1:nIter
    for i = 1:nParticles
        Kp = pos(i,1);
        Ki = pos(i,2);
        Kd = pos(i,3);
        
        % Προσομοίωση συστήματος
        y = zeros(1, N);
        u = zeros(1, N);
        integral = 0;
        prev_error = 0;
        y(1) = 0;
        
        for k = 2:N
            error = w_ref(k) - y(k-1);
            integral = integral + error * dt;
            derivative = (error - prev_error) / dt;
            prev_error = error; % Ανανέωση προηγούμενου σφάλματος
            
            % Υπολογισμός ψηφιακού PID
            u(k) = Kp*error + Ki*integral + Kd*derivative;
            
            % Μοντέλο κινητήρα (Εξίσωση Euler)
            y(k) = y(k-1) + dt * ((D/Cm)*u(k) - (Kl/Cm)*y(k-1));
        end
        
        % Υπολογισμός Κόστους
        cost = sum((w_ref - y).^2);
        
        % Ενημέρωση Personal Best
        if cost < pbest_cost(i)
            pbest_cost(i) = cost;
            pbest(i,:) = pos(i,:);
        end
        
        % Ενημέρωση Global Best
        if cost < gbest_cost
            gbest_cost = cost;
            gbest = pos(i,:);
        end
    end
    
    % Ενημέρωση Ταχύτητας και θέσης Σωματιδίων
    for i = 1:nParticles
        r1 = rand(1,3);
        r2 = rand(1,3);
        
        % Εξίσωση ταχύτητας PSO
        vel(i,:) = w_pso*vel(i,:) + c1*r1.*(pbest(i,:) - pos(i,:)) + c2*r2.*(gbest - pos(i,:));
        pos(i,:) = pos(i,:) + vel(i,:);
        
        % Περιορισμός στα όρια
        pos(i,1) = min(max(pos(i,1), Kp_bounds(1)), Kp_bounds(2));
        pos(i,2) = min(max(pos(i,2), Ki_bounds(1)), Ki_bounds(2));
        pos(i,3) = min(max(pos(i,3), Kd_bounds(1)), Kd_bounds(2));
    end
    
    fprintf('PSO iter %2d | Best cost = %.4f\n', iter, gbest_cost);
end

% Εκτύπωση αποτελεσμάτων
Kp_opt = gbest(1);
Ki_opt = gbest(2);
Kd_opt = gbest(3);
fprintf('\n Βελτιστοποιημένα Κέρδη PID (PSO) \n');
fprintf('Kp = %.3f\nKi = %.3f\nKd = %.3f\n', Kp_opt, Ki_opt, Kd_opt);


% 6. Τελική Προσομοίωση με το Βέλτιστο PID
y = zeros(1, N);
u = zeros(1, N);
integral = 0;
prev_error = 0;

for k = 2:N
    error = w_ref(k) - y(k-1);
    integral = integral + error * dt;
    derivative = (error - prev_error) / dt;
    prev_error = error;
    
    u(k) = Kp_opt*error + Ki_opt*integral + Kd_opt*derivative;
    y(k) = y(k-1) + dt * ((D/Cm)*u(k) - (Kl/Cm)*y(k-1));
end

% 7. Γραφήματα (Plots)
figure('Name', 'PSO Optimized Control', 'NumberTitle', 'off');

subplot(2,1,1);
plot(time, y, 'b', 'LineWidth', 1.5); hold on;
plot(time, w_ref, 'r--', 'LineWidth', 1.5);
ylabel('Ταχύτητα \omega (rad/s)');
title('Απόκριση Υδραυλικού Κινητήρα με PSO PID');
legend('Έξοδος Συστήματος', 'Σήμα Αναφοράς (w=1)', 'Location', 'southeast');
grid on;

subplot(2,1,2);
plot(time, u, 'g', 'LineWidth', 1.5);
ylabel('Σήμα Ελέγχου (u)');
xlabel('Χρόνος (sec)');
grid on;