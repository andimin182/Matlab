%% EL Dynamic modelisation using the MOVING-FRAME algorithm
%--------- 2 November 2020 by Andi -----------------------------


clear all
close all
clc

%-- Run the DH file to compute the rotation matrices
%dh_parameters

clear all
clc

load('A_matrices')

%% Extract Rotation matrices and position of RFi-1 to RFi expressed in RFi-1

for i = 1 : num_link
    % Rotation matrix from i-1 to i
    eval(['R' num2str(i) ' = A' num2str(i) '(1:3,1:3)'])
    % position of RF i expressed in i-1
    eval(['r' num2str(i) ' = A' num2str(i) '(1:3,4)'])
end
 
%% Create joint variables and joint type sigma 0 for revolute and 1 for prismatic
Q = sym('q', [1 num_link]);
Q_dot = sym('q_dot', [1 num_link]);

%% Mass of links
m = sym('m', [1 num_link]);

%% Inertia
I = sym('Izz', [1 num_link]);


%% Matrix of position of COM i expressed in frame i
Rc = sym('Rc', [3 num_link]);

% Matrix of position of COM i expressed in frame 0
R0c = sym('R0c', [3 num_link]);

%% Initialitation
v0 = [0; 0; 0];
omega0 = [0; 0; 0];
z = [0;0;1];

%% Moving frame Algorithm


% Step 1 compute linear, angular velocities and kinetic energy of each link
for i= 1 : num_link
    % Compute omega_i expressed in frame i
    eval(['omega' num2str(i) '=transpose(R' num2str(i) ')*[omega' num2str(i-1) '+Q_dot(' num2str(i) ')*z]'])
    % Compute v_i expressed in frame i
    eval(['v' num2str(i) '= transpose(R' num2str(i) ')*v' num2str(i-1) '+cross(omega' num2str(i) ',transpose(R' num2str(i) ')*r' num2str(i) ')'])
    % Compute vc_i expressed in frame i
    eval(['vc' num2str(i) '=v' num2str(i) '+cross(omega' num2str(i) ',Rc(:,' num2str(i) '))'])
    % Compute kinetic energy
    eval(['T' num2str(i) ' =(1/2)*m(' num2str(i) ')*transpose(vc' num2str(i) ')*vc' num2str(i) '+(1/2)*I(' num2str(i) ')*transpose(omega' num2str(i) ')*omega' num2str(i)])
end


%% This part isn't universal
% Step 2 compute total kinetic energy
syms l1 l2 l3 d1 d2 d3
L = sym('l', [1 num_link]);
D = sym('d', [1 num_link]);


T = simplify(T1 + T2);
T = subs(T, Rc, [-L(1)+D(1) -L(2)+D(2); 0 0; 0 0;]);

m11 = simplify(subs(T*2, [Q_dot(1) Q_dot(2)], [1 0]));
m22 = simplify(subs(T*2, [Q_dot(1) Q_dot(2)], [0 1]));
m12 = simplify(subs(T*2-m11-m22, [Q_dot(1) Q_dot(2)], [1 1]));
m21 = m12;

% INERTIA MATRIX

M = [ m11 m12; m21 m22];

% Centrifugal & Coriolis terms
C1 = (1/2)*[[diff(M(:,1), Q(1))  diff(M(:,1), Q(2))] + transpose([diff(M(:,1), Q(1))  diff(M(:,1), Q(2))])- diff(M, Q(1))];
C2 = (1/2)*[[diff(M(:,2), Q(1))  diff(M(:,2), Q(2))] + transpose([diff(M(:,2), Q(1))  diff(M(:,2), Q(2))])- diff(M, Q(2))];

c1 = Q_dot*C1*transpose(Q_dot);
c2 = Q_dot*C2*transpose(Q_dot);

c = [c1; c2];
% Potential energy
g0 = 9.81;

g = [0; -g0; 0];
U1 = -m(1)*transpose(g)*R0c(:,1);
U2 = -m(2)*transpose(g)*R0c(:,2);

U = simplify(U1 +U2);
G = [diff(subs(U, R0c, [0 0; d1*sin(Q(1)) l1*sin(Q(1))+d2*sin(Q(1)+Q(2)); 0 0 ]),Q(1)) diff(subs(U, R0c, [0 0; d1*sin(Q(1)) l1*sin(Q(1))+d2*sin(Q(1)+Q(2)); 0 0 ]),Q(2))];



%save('EL_Dynamic_model', 'M', 'c', 'G', 'Q', 'Q_dot', 'L', 'D', 'I', 'm')


