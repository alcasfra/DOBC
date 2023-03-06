% =========================================================================
% This software is licensed under the Creative Commons Attribution (CC BY)
% license, which allows others to distribute, remix, adapt, and build upon
% the work, even for commercial purposes, as long as they give appropriate
% credit to the original author.
%
% The author of this software is Alberto Castillo (alcasfra@gmail.com).
% The software is provided as-is, without warranty of any kind, express or
% implied, including but not limited to the warranties of merchantability,
% fitness for a particular purpose and noninfringement. In no event shall
% the author be liable for any claim, damages or other liability, whether
% in an action of contract, tort or otherwise, arising from, out of or in
% connection with the software or the use or other dealings in the software.
%
% If you have any questions or comments about this software, please
% contact the author at alcasfra@gmail.com.
% =========================================================================


clear all
close all
clc

% Load files
sim = {};
system = "model1_results";
number_of_files = 1000;
for i=1:1:number_of_files

    % Define data file
    data_file = [system "/sim" i ".mat"];

    % Plot
    clc
    disp(join(["loading data file " system "/sim" i ".mat..."],""))

    % Load data
    sim{i} = load(join([system "/sim" i ".mat"],""),'results');
    
end


%% Get statistical results
LQR_deviation_from_optimum = zeros(number_of_files,1);
r0_deviation_from_optimum  = zeros(number_of_files,1);
r1_deviation_from_optimum  = zeros(number_of_files,1);
r2_deviation_from_optimum  = zeros(number_of_files,1);
LQR_state_traj_mean_error  = zeros(number_of_files,1);
r0_state_traj_mean_error   = zeros(number_of_files,1);
r1_state_traj_mean_error   = zeros(number_of_files,1);
r2_state_traj_mean_error   = zeros(number_of_files,1);
LQR_control_traj_mean_error  = zeros(number_of_files,1);
r0_control_traj_mean_error   = zeros(number_of_files,1);
r1_control_traj_mean_error   = zeros(number_of_files,1);
r2_control_traj_mean_error   = zeros(number_of_files,1);
O_0   = zeros(number_of_files,1);
O_1   = zeros(number_of_files,1);
O_2   = zeros(number_of_files,1);
for i=1:1:number_of_files

    % deviations from the optimum
    LQR_deviation_from_optimum(i) = (sim{i}.results.J_LQR - sim{i}.results.J_opt)/sim{i}.results.J_opt*100;
    r0_deviation_from_optimum(i) = (sim{i}.results.J_r0 - sim{i}.results.J_opt)/sim{i}.results.J_opt*100;
    r1_deviation_from_optimum(i) = (sim{i}.results.J_r1 - sim{i}.results.J_opt)/sim{i}.results.J_opt*100;
    r2_deviation_from_optimum(i) = (sim{i}.results.J_r2 - sim{i}.results.J_opt)/sim{i}.results.J_opt*100;

    % deviations from the optimal state trajectories
    xopt = sim{i}.results.xopt.signals.values;
    xlqr = sim{i}.results.xlqr.signals.values;
    xr0 = sim{i}.results.xr0.signals.values;
    xr1 = sim{i}.results.xr1.signals.values;
    xr2 = sim{i}.results.xr2.signals.values;
    LQR_state_traj_mean_error(i) = norm(mean(abs((xopt-xlqr)./(max(xopt)-min(xopt))*100)));
    r0_state_traj_mean_error(i)  = norm(mean(abs((xopt-xr0)./(max(xopt)-min(xopt))*100)));
    r1_state_traj_mean_error(i)  = norm(mean(abs((xopt-xr1)./(max(xopt)-min(xopt))*100)));
    r2_state_traj_mean_error(i)  = norm(mean(abs((xopt-xr2)./(max(xopt)-min(xopt))*100)));

    % deviations from the optimal control actions
    uopt = sim{i}.results.uopt.signals.values;
    ulqr = sim{i}.results.ulqr.signals.values;
    ur0 = sim{i}.results.ur0.signals.values;
    ur1 = sim{i}.results.ur1.signals.values;
    ur2 = sim{i}.results.ur2.signals.values;
    LQR_control_traj_mean_error(i) = norm(mean(abs((uopt-ulqr)./(max(uopt)-min(uopt))*100)));
    r0_control_traj_mean_error(i)  = norm(mean(abs((uopt-ur0)./(max(uopt)-min(uopt))*100)));
    r1_control_traj_mean_error(i)  = norm(mean(abs((uopt-ur1)./(max(uopt)-min(uopt))*100)));
    r2_control_traj_mean_error(i)  = norm(mean(abs((uopt-ur2)./(max(uopt)-min(uopt))*100)));

    % Or
    O_0(i,:) = mean(vecnorm(sim{i}.results.O0.signals.values')');
    O_1(i,:) = mean(vecnorm(sim{i}.results.O1.signals.values')');
    O_2(i,:) = mean(vecnorm(sim{i}.results.O2.signals.values')');

end

% Computation of means and standard deviations for each variable
LQR_deviation_from_optimum_mean = mean(LQR_deviation_from_optimum);
LQR_deviation_from_optimum_std  = std(LQR_deviation_from_optimum);
r0_deviation_from_optimum_mean = mean(r0_deviation_from_optimum);
r0_deviation_from_optimum_std  = std(r0_deviation_from_optimum);
r1_deviation_from_optimum_mean = mean(r1_deviation_from_optimum);
r1_deviation_from_optimum_std  = std(r1_deviation_from_optimum);
r2_deviation_from_optimum_mean = mean(r2_deviation_from_optimum);
r2_deviation_from_optimum_std  = std(r2_deviation_from_optimum);

LQR_state_traj_mean_error_mean = mean(LQR_state_traj_mean_error);
LQR_state_traj_mean_error_std  = std(LQR_state_traj_mean_error);
r0_state_traj_mean_error_mean = mean(r0_state_traj_mean_error);
r0_state_traj_mean_error_std  = std(r0_state_traj_mean_error);
r1_state_traj_mean_error_mean = mean(r1_state_traj_mean_error);
r1_state_traj_mean_error_std  = std(r1_state_traj_mean_error);
r2_state_traj_mean_error_mean = mean(r2_state_traj_mean_error);
r2_state_traj_mean_error_std  = std(r2_state_traj_mean_error);

LQR_control_traj_mean_error_mean = mean(LQR_control_traj_mean_error);
LQR_control_traj_mean_error_std  = std(LQR_control_traj_mean_error);
r0_control_traj_mean_error_mean = mean(r0_control_traj_mean_error);
r0_control_traj_mean_error_std  = std(r0_control_traj_mean_error);
r1_control_traj_mean_error_mean = mean(r1_control_traj_mean_error);
r1_control_traj_mean_error_std  = std(r1_control_traj_mean_error);
r2_control_traj_mean_error_mean = mean(r2_control_traj_mean_error);
r2_control_traj_mean_error_std  = std(r2_control_traj_mean_error);

O_0_mean = mean(O_0);
O_0_std = std(O_0);
O_1_mean = mean(O_1);
O_1_std = std(O_1);
O_2_mean = mean(O_2);
O_2_std = std(O_2);
