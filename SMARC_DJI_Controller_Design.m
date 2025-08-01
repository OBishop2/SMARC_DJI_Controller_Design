clear 
clc

%% Calculates Moment of Inertia matrix from values given in Unity; used for excel math


I_diag = [0.01748696 0 0; 0 0.02578914 0; 0 0 0.0326721];
euler_deg = [-2.4*10^-5 179.9998 353.8483];

euler_rad = deg2rad(euler_deg);

Rx = @(theta) [1, 0, 0;
               0, cos(theta), -sin(theta);
               0, sin(theta),  cos(theta)];

Ry = @(theta) [cos(theta), 0, sin(theta);
               0, 1, 0;
              -sin(theta), 0, cos(theta)];

Rz = @(theta) [cos(theta), -sin(theta), 0;
               sin(theta),  cos(theta), 0;
               0, 0, 1];

R = Rz(euler_rad(3)) * Rx(euler_rad(1)) * Ry(euler_rad(2));

I = R * I_diag * R'

%% Solves system of equations for prop speeds given torques, forces, and lengths of arms, assuming 0 yaw.


syms F Tx Ty w1 w2 w3 w4 l1 l2 l3 l4
A = [1 1 1 1 F; -w1 w2 w3 -w4 Tx; l1 l2 -l3 -l4 Ty; 1 -1 1 -1 0];
refA = rref(A)
simplify(refA(1, 5) + refA(2, 5) + refA(3, 5) + refA(4, 5))

%% Altitude Controller
%This is where the actual control design for the altitude controller
%occurs. The process applied here is design by emulation using a lead
%controller
period = .002;
rpm_ratio = .006;
num_props = 4;
mass = 6.47;

%This is the approximate plant of the controller; 
%This assumes that the props are spinning fast enough to be at a constant 
%height and then uses the mass and rpm to force ratio to determine the acceleration from the props
P = tf([rpm_ratio * num_props / mass], [1 0 0]) 

w_cg = 1; %W_cg = 1.8 / t_r
[mag, phase] = bode(P, w_cg);
figure(1)
margin(P)

K = mag^-1 %Necessary gain to reach desired crossover
figure(2)
margin(K*P)

%The amount by which the phase margin is increased. In this case, the base 
%phase margin is 0, so this value is roughly the phase margin. Higher phase
%margin is more stable with a smaller overshoot.
phi_max = 55
alpha = (1 - sin(deg2rad(phi_max))) / (1 + sin(deg2rad(phi_max)))

%Calculate pole and zero of continous time controller
z = sqrt(alpha) * w_cg
p = w_cg / sqrt(alpha)

C = K * tf([1/z 1], [1/p 1])
figure(3)
margin(C*P)

%Converts continuos controller into digital controller. Use this as a
%starting point and then tune from there
D = c2d(C, period, "zoh")

%% Pitch Controller
period = .002;
l = 0.29488; %length from cOM to propellors
R = 0.006; %rpm to force multiplier

I_p = 0.0256938016305383; %Moment of inertia about the pitch axis

P_pitch = tf([l*R], [I_p 0 0]); %Rotation plant
w_cg = .7;

[mag, phase] = bode(P_pitch, w_cg);
K_pitch = mag ^ -1
figure(1)
margin(K_pitch*P_pitch)

phi_max = 85
alpha = (1 - sin(deg2rad(phi_max))) / (1 + sin(deg2rad(phi_max)))

z = sqrt(alpha) * w_cg
p = w_cg / sqrt(alpha)

C = K_pitch * tf([1/z 1], [1/p 1])
figure(2)
margin(C*P_pitch)

D = c2d(C, period, "zoh")

%% Roll Controller
period = .002;
w = 0.3991; %length from COM to propellors
R = 0.006; %rpm to force multiplier

I_r = 0.017582298; %Moment of inertia about the pitch axis

P_roll = tf([w*R], [I_r 0 0]); %Rotation plant
w_cg = 1.2;

[mag, phase] = bode(P_roll, w_cg);
K_roll = mag ^ -1
figure(2)
margin(K_roll*P_roll)

phi_max = 65
alpha = (1 - sin(deg2rad(phi_max))) / (1 + sin(deg2rad(phi_max)))

z = sqrt(alpha) * w_cg
p = w_cg / sqrt(alpha)

C = K_roll * tf([1/z 1], [1/p 1])
figure(2)
margin(C*P_roll)

D = c2d(C, period, "zoh")

%% Velocity Controller, Assuming Attitude is "Fast"
period = .002;
g = 9.81;

P_vel = g * pi / 180 * tf([1], [1 0 ])
figure(1)
margin(P_vel)

w_cg = 3;
[mag, phase] = bode(P_vel, w_cg);
K_vel = mag ^ -1
figure(2)
margin(K_vel*P_vel)

phi_max = 15
alpha = (1 - sin(deg2rad(phi_max))) / (1 + sin(deg2rad(phi_max)))

z = sqrt(alpha) * w_cg
p = w_cg / sqrt(alpha)

C = K_vel * tf([1/z 1], [1/p 1])
figure(3)
margin(C*P_vel)

D = c2d(C, period, "zoh")
