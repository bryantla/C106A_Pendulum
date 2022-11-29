% parameters
M = 1.00;           % mass of cart [kg]
m = 0.0698;         % mass of pendulum [kg]
Lp = 0.9144/2;      % pendulum distance from pivot to center of mass [m]
Ic = m*(Lp^2)/3;    % moment of inertia of pendulum about its center
Ie = 4*m*(Lp^2)/3;  % moment of inertia of pendulum about its end
g = 9.81;           % gravitational acceleration [m/s^2]

% state-space equations
den = Ic*(M+m)+M*m*Lp^2; % denominator for the A and B matrices

A = [0 1 0 0; 0 0 (m^2*g*Lp^2)/den 0; 0 0 0 1; 0 0 m*g*Lp*(M+m)/den 0];
B = [0; (Ic+m*Lp^2)/den; 0; m*Lp/den];
C = [1 0 0 0; 0 0 1 0];
D = [0; 0];

open_loop_poles = eig(A)
sys = ss(A,B,C,D);

% controller design
% closed-loop system matrix
syms k1 k2 k3 k4 s
K = [k1 k2 k3 k4];
Ak = A - B*K;
vpa(Ak,3)

% characteristic polynomial
P = det(s*eye(4) - Ak);
vpa(P,3)

% desired characteristic polynomial
Pdes = (s-(-2+10*1i))*(s-(-2-10*1i))*(s-(-1.6+1.3*1i))*(s-(-1.6-1.3*1i));
vpa(simplify(Pdes))

% state feedback, pole placement
p = [-2+10*1i; -2-10*1i; -1.6+1.3*1i; -1.6-1.3*1i];
K_ = place(A,B,p);
K = K_

% LQR
Q = C'*C;
R = 1;
[K,S,P] = lqr(A,B,Q,R)

% step response
Ak = A - B*K;
TF = tf(ss(Ak,B*K,C,0));
TF2 = TF(2,1);
figure
step(TF2)
