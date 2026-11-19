% =========================================================================
% Analyse des performances du Contrôleur Flou - Robot 2-DOF
% =========================================================================
clc; clear; close all;

% --- 1. Chargement des données ---
% Lecture du fichier texte généré par le programme C
data = load('data.txt');

t = data(:, 1);         % Temps
th1_d = data(:, 2);     % Position désirée Articulation 1
th1 = data(:, 3);       % Position réelle Articulation 1
th2_d = data(:, 4);     % Position désirée Articulation 2
th2 = data(:, 5);       % Position réelle Articulation 2

% --- 2. Plot: Joint 1 Position Tracking ---
figure('Name', 'Joint 1 Tracking', 'Color', 'w');
plot(t, th1_d, 'r--', 'LineWidth', 2); hold on;
plot(t, th1, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Position (rad)', 'FontWeight', 'bold');
title('Joint 1 Position Tracking');
legend('Desired trajectory (\theta_{1d})', 'Actual position (\theta_1)', 'Location', 'best');
grid on;

% --- 3. Plot: Joint 2 Position Tracking ---
figure('Name', 'Joint 2 Tracking', 'Color', 'w');
plot(t, th2_d, 'r--', 'LineWidth', 2); hold on;
plot(t, th2, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Position (rad)', 'FontWeight', 'bold');
title('Joint 2 Position Tracking');
legend('Desired trajectory (\theta_{2d})', 'Actual position (\theta_2)', 'Location', 'best');
grid on;

% --- 4. Error calculation for discussion ---
% Calculation of Root Mean Square Error (RMSE) over the entire simulation
rmse1 = sqrt(mean((th1_d - th1).^2));
rmse2 = sqrt(mean((th2_d - th2).^2));

fprintf('--- Overall Accuracy Analysis ---\n');
fprintf('Average tracking error (Joint 1) : %.4f rad\n', rmse1);
fprintf('Average tracking error (Joint 2) : %.4f rad\n', rmse2);