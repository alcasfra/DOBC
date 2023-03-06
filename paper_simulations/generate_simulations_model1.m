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
%
% The folder 'aux_functions' should be added to the MatLab path in order to
% execute this script.
% =========================================================================


clear all
close all
clc

% Path where the simulation results will be stored
results_path = "model1_results";

% Nominal model {A,B1,B2,C,D}
x0 = [0, 0, 0];
A  = [-1.4, 0.2, -0.1
      -0.2, -0.8, -0.3
      0.1 -0.1 -0.9];
B2 = [0.1, 0.8
      1.1, 0.3
      0.9, 0.5];
B1 = [1;1;1];
C = eye(3);
D = zeros(3,1);
n = 3; m=2; q=1; p=3;

% Build observer {bA, bB2, bC2} - Eq.(13)
r = 2;
l = n+(r+1)*q;
Pi = [eye(q) zeros(q,r*q)];
Phi = [zeros(r*q,q) eye(r*q);
       zeros(q,q)   zeros(q,r*q)];

bA = [A                 B1*Pi;
      zeros((r+1)*q,n)   Phi];
bB2 = [B2; zeros((r+1)*q,m)];
bC2 = [C D zeros(p,r*q)];

% Get observer gain
Obs_poles = -40;
L = place(bA', bC2', Obs_poles*[1.01 1.02 0.99 0.98 0.97 0.96])';
% eig(bA-L*bC2)

% Cost matrices
Q = eye(3);
R = 0.1*eye(2);

% Control-law matrices - Eq.(2)
S_T = care(A,B2,Q,R); % Solve ARE Eq.(6).
Kx = -inv(R)*B2'*S_T;
Kw0 = -inv(R)*B2'*(-(A+B2*Kx)')^(-1)*S_T*B1;
Kw1 = -inv(R)*B2'*(-(A+B2*Kx)')^(-2)*S_T*B1;
Kw2 = -inv(R)*B2'*(-(A+B2*Kx)')^(-3)*S_T*B1;
% Kw3 = (-1)^3*inv(R)*B2'*((A+B2*Kx)')^(-4)*S_T*B1;
% [...]

% Display the norm of the residual Or (omitting the delta_r(t) term). It is
% useful to anticipate if the optimality error is going to be low. If these
% matrices rapidly decrease with 'r', it is a good indicator.
Acl = (A+B2*Kx);
disp("eig(Acl) =")
disp(eig(Acl))
disp("-inv(R)*B2'*(-Acl')^(-(1)) =")
disp(-inv(R)*B2'*(-Acl')^(-(1)))
disp("-inv(R)*B2'*(-Acl')^(-(2)) =")
disp(-inv(R)*B2'*(-Acl')^(-(2)))
disp("-inv(R)*B2'*(-Acl')^(-(3)) =")
disp(-inv(R)*B2'*(-Acl')^(-(3)))

% Compute the time instant 'h' that renders expm(Acl'*h)*S_T*B1 almost
% null. Needed for the exact trapezoidal integration of Eq.(8)
global h
h = 2.5; 
% norm(expm(Acl'*0)*S_T*B1)
% norm(expm(Acl'*h)*S_T*B1)

% Define the number of intervals for the trapezoidal integration of Eq.(8)
global delta
delta = h/300;

% Generate a batch of N=1000 simulations and save the results.
numer_of_simulations = 1000;
J = {};
O_0 = {};
O_1 = {};
O_2 = {};
xopt = {};
xlqr = {};
xr0 = {};
xr1 = {};
xr2 = {};
uopt = {};
ulqr = {};
ur0 = {};
ur1 = {};
ur2 = {};
data_init = 1;
for i=data_init:1:data_init+numer_of_simulations

    % Plot progress
    clc
    disp(["running simulation: ", i])

    % Generate random disturbance
    global a0 b0 a1 b1 a2 b2 w0 w1 w2
    ramdom_sample = rand(9,1)*5;
    a0 = ramdom_sample(1);
    a1 = ramdom_sample(2);
    a2 = ramdom_sample(3);
    b0 = ramdom_sample(4);
    b1 = ramdom_sample(5);
    b2 = ramdom_sample(6);
    w0 = ramdom_sample(7);
    w1 = ramdom_sample(8);
    w2 = ramdom_sample(9);
    
    % Define observer initial state - we do not want interferences caused
    % by the observer initial transient response.
    eta0 = [x0,b0+b1+b2,a0*w0+a1*w1+a2*w2,-b0*w0^2-b1*w1^2-b1*w1^2];

    % Simulate
    sim('simulator_with_observer')

    % Save results
    results = struct();
    results.w = dist;
    results.J_opt = J_opt.signals.values(end);
    results.J_LQR = J_LQR.signals.values(end);
    results.J_r0 = J_r0.signals.values(end);
    results.J_r1 = J_r1.signals.values(end);
    results.J_r2 = J_r2.signals.values(end);
    results.O0 = O0;
    results.O1 = O1;
    results.O2 = O2;
    results.xopt = x_opt;
    results.xlqr = x_LQR;
    results.xr0 = x_r0;
    results.xr1 = x_r1;
    results.xr2 = x_r2;
    results.uopt = u_opt;
    results.ulqr = u_LQR;
    results.ur0 = u_r0;
    results.ur1 = u_r1;
    results.ur2 = u_r2;

    save(join([system "/sim" i ".mat"],""),'results')

end
