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
% The folder 'Yalmip_and_Sedumi' should be added to the MatLab path in
% order to execute this script.
% =========================================================================


% Application to the benchmark magnetic levitation control problem proposed
% in:
% =========================================================================
% Yang, J., Zolotas, A., Chen, W. H., Michail, K., & Li, S. (2011). Robust
% control of nonlinear MAGLEV suspension system with mismatched
% uncertainties via DOBC approach. ISA transactions, 50(3), 389-396.
% =========================================================================

clear all
close all
clc


%% MAGLEV linearized model: Considering linearization residuals and carriage mass variations.
% Parameters.
%%%%%%%%%%%%%%%
mc = 1000; %kg. Carriage mass.
Rc = 10; %Ohms. Coil's resistance.
Lc = 0.1; %H. Coil's inductance.
Nc = 2000; % Number of turns.
Ap = 0.01; %m^2. Pole face area.
Kb = 0.0015; % Flux constant.
Kf = 9810;

% Operating point
%%%%%%%%%%%%%%%
F0 = mc*9.81; %N. Fuerza = peso del tren.
G0 = 0.015; %m. Airgap.
B0 = sqrt(F0/Kf); %T. Flux density.
I0 = (1/Kb)*B0*G0; %A. Current.
V0 = I0*Rc; %V. Input Voltage.

% Nominal matrices.
%%%%%%%%%%%%%%%
a11 = -Rc/(Lc+(Kb*Nc*Ap)/G0);
a12 = -(Kb*Nc*Ap*I0)/(G0^2*(Lc+(Kb*Nc*Ap)/G0));
a21 = -2*Kf*Kb^2*I0/(mc*G0^2);
a23 = 2*Kf*Kb^2*I0^2/(mc*G0^3);
a32 = -1;
b2_11 = 1/(Lc+(Kb*Nc*Ap)/G0);
b1_11 = (Kb*Nc*Ap*I0)/(G0^2*(Lc+(Kb*Nc*Ap)/G0));

A = [a11 a12 0;a21 0 a23;0 a32 0];
B2 = [b2_11;0;0];
B1 = eye(3);
C2 = eye(3);
D21 = zeros(3);

n = 3; m = 1; q = 3; p = 3;

% Disturbance - change of slope
%%%%%%%%%%%%%%
tsim = 7;
h = 0.01;
line1 = 0:h:0.5;
line2 = 0.5*ones(1,1/h);
line3 = 0.5:-h:0;

time = 0:h:2*tsim;
ddw = [line1 line2 line3 zeros(1,(2*tsim-2-h)/h)];
sim('generateDisturbance')

tw = time(1:length(time)/2)';
w_t = z_t(1:length(time)/2);
dw_t = dz_t(1:length(time)/2);
ddw_t = ddz_t(1:length(time)/2);


%% LQ-DOBC design
C1 = [1 0 0;0 1 0;0 0 1];
R = [1];
gamma = 0.1;
r = 2;

% Get control law matrices
[Kx, Kw] = getKxKw(A,B1,B2,C1,R,gamma,r);
K = [Kx Kw];


%% DOB design - uses Linear Matrix Inequalities to optimize the gain.
% Get extended matrices
n = size(A,1); m = size(B2,2); q = size(B1,2); p = size(C2,1);
l = n+(r+1)*q;

Pi  = [eye(q) zeros(q,r*q)];
Phi = [zeros(r*q,q) eye(r*q);
       zeros(q,q)   zeros(q,r*q)];

bA  = [A                 B1*Pi;
       zeros((r+1)*q,n)   Phi];
bB2 = [B2; zeros((r+1)*q,m)];
bC2 = [C2 zeros(p,(r+1)*q)];
bB1 = [zeros(q,n+r*q) eye(q)]';

% Optimize L.
delta = 20; % To optimize
radius = 20.5;        
center = 0;
M = [0 1;
      0 0];
N = [-radius   center
      center   -radius];
  
[L,alpha] = optimizeL(A,B1,B2,C2,r,K,M,N,delta);


%% Simulation
%%%%%%%%%%%%%%
% r = 0
Kfb = K*[eye(6) zeros(6);zeros(6) zeros(6)]; % u = Kfb*\eta = Kx*x + Kw*w
sim('OptimalSol') % Variables: x_OC, u_OC.
x_OC1 = x_OC;
u_OC1 = u_OC;

% r = 1
Kfb = K*[eye(9) zeros(9,3);zeros(3,9) zeros(3)]; % u = Kfb*\eta = Kx*x + Kw0*w + Kw1*dw
sim('OptimalSol') % Variables: x_OC, u_OC.
x_OC2 = x_OC;
u_OC2 = u_OC;

% r = 2
Kfb = K; % u = Kfb*\eta = Kx*x + Kw0*w + Kw1*dw + Kw2*ddw
sim('OptimalSol') % Variables: x_OC, u_OC.
x_OC3 = x_OC;
u_OC3 = u_OC;

% Original solution Yang2011 - ISA 
KxY = Kx;
KdY = -inv([0 0 1]*inv(A+B2*Kx)*B2)*[0 0 1]*inv(A+B2*Kx)*B1;
LY = [40 0 0
     0 40 0
     0 0 40];
sim('Yang2011') % Variables: x_Yang, u_Yang.


% Disturbance cancelation policy
sim('GESO') % Variables: x_Yang, u_Yang.


%% PLOT
% configuration parameters
screensize = get(0,'ScreenSize'); screenwidth = screensize(3); screenheight = screensize(4); %get screen dimensions.
x_position = screenwidth/3;   %screen x-y position
y_position = screenheight/15;
xyRatio = 0.7;      %x-y ratio (ej: 1920x1080)
x_dimension = screenwidth*0.65;  %screen x dimension (pixels)
text_size = 15;         %size of text
legend_text_size = 15;  %size of text in the legend
line_width = 2;         %line width

% setting configuration
fig = figure('Position', [x_position, y_position, x_dimension, xyRatio*x_dimension]); %Saca la figura en la posición de la pantalla que se ha puesto con sus dimensiones.
set(0, 'defaulttextInterpreter','latex'); %Interprete latex.

%%%%%%%%%%%%
% plot lines
%%%%%%%%%%%%
subplot(211)
line(x_Yang.time,x_Yang.signals.values(:,3),'Color',[0.45 0.26 0.26],'LineWidth',line_width,'LineStyle','-.');    grid on;
line(x_GESO.time,x_GESO.signals.values(:,3),'Color',[0 0.5 0],'LineWidth',line_width,'LineStyle','-');    grid on;
line(x_OC1.time,x_OC1.signals.values(:,3),'Color',[0.3 0 0.4],'LineWidth',line_width,'LineStyle','--');
%     line(x_OC2.time,x_OC2.signals.values(:,3),'Color',[0.15 0 0.7],'LineWidth',line_width,'LineStyle','-');
line(x_OC3.time,x_OC3.signals.values(:,3),'Color',[0 0 1],'LineWidth',line_width,'LineStyle','-');
grid on;
l = legend('Yang et.al.','Li et.al','Proposed design $(r=0)$','Proposed design $(r=2)$',...
    'Location','northeast','Orientation','vertical');           
set(l,'Interpreter','latex','FontSize',legend_text_size);
ejey = ylabel('$\Delta(z_t-z)$ (m)');
xlim([0 4])
ylim([-0.001 0.005])
box on
set(gca,'fontsize',text_size);

subplot(212)
line(u_Yang.time,u_Yang.signals.values,'Color',[0.45 0.26 0.26],'LineWidth',line_width,'LineStyle','-.');    grid on;
line(u_GESO.time,u_GESO.signals.values,'Color',[0 0.5 0],'LineWidth',line_width,'LineStyle','-');    grid on;
line(u_OC1.time,u_OC1.signals.values,'Color',[0.3 0 0.4],'LineWidth',line_width,'LineStyle','--');
%     line(u_OC2.time,u_OC2.signals.values,'Color',[0.15 0 0.7],'LineWidth',line_width,'LineStyle','-');
line(u_OC3.time,u_OC3.signals.values,'Color',[0 0 1],'LineWidth',line_width,'LineStyle','-');
grid on
ejex = xlabel('t (sec)');
ejey = ylabel('$\Delta V_c(t)$ (V)');
xlim([0 4])
ylim([-5 30])
box on
set(gca,'fontsize',text_size);


%     % save figure
%     print(fig,'zt_z','-depsc','-r300');


%% PLOT
% configuration parameters
screensize = get(0,'ScreenSize'); screenwidth = screensize(3); screenheight = screensize(4); %get screen dimensions.
x_position = screenwidth/3;   %screen x-y position
y_position = screenheight/15;
xyRatio = 0.27;      %x-y ratio (ej: 1920x1080)
x_dimension = screenwidth*0.5;  %screen x dimension (pixels)
text_size = 14;         %size of text
legend_text_size = 14;  %size of text in the legend
line_width = 1;         %line width

% setting configuration
fig = figure('Position', [x_position, y_position, x_dimension, xyRatio*x_dimension]); %Saca la figura en la posición de la pantalla que se ha puesto con sus dimensiones.
set(0, 'defaulttextInterpreter','latex'); %Interprete latex.

%%%%%%%%%%%%
% plot lines
%%%%%%%%%%%%
line(time,z_t,'Color','b','LineWidth',line_width,'LineStyle','-');
line(time,dz_t,'Color','k','LineWidth',line_width,'LineStyle',':');    grid on;
line(time,ddz_t,'Color',[0 0.5 0],'LineWidth',line_width,'LineStyle','--');    grid on;
l = legend('$z_t$','$\dot{z}_t$','$\ddot{z}_t$','Location','northeast','Orientation','horizontal');           
set(l,'Interpreter','latex','FontSize',legend_text_size);
ejex = xlabel('t (sec)');
ejey = ylabel('rail´s position');
xlim([0 7])
ylim([0 1.5])
box on
set(gca,'fontsize',text_size);


%     % save figure
%     print(fig,'disturbance','-depsc','-r300');