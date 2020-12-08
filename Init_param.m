%% -------------- Init data ----------------------------
clear all
close all
clc

load('Dynamic_model')

% Link 1
l1 = 0.145;
d1 = l1/2;
m1 = 1.174;
Iz1 = 0.002;

% Link 2
l2 = 0.145;
d2 = l2/2;
m2 = 1.1134;
Iz2 = 0.00023;

%% Subs values
M = subs(M, [L(1) L(2) m(1) m(2) I(1) I(2) D(1) D(2)], [l1 l2 m1 m2 Iz1 Iz2 d1 d2]);
c = subs(c, [L(1) L(2) m(1) m(2) I(1) I(2) D(1) D(2)], [l1 l2 m1 m2 Iz1 Iz2 d1 d2]);
G = transpose(subs(G, [L(1) L(2) m(1) m(2) I(1) I(2) D(1) D(2)], [l1 l2 m1 m2 Iz1 Iz2 d1 d2]));