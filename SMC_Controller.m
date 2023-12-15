function [u] = SMC_Controller(z, z_d, p, control_mode)

% Define intermediate control inputs w1, w2, w3, w4
%   w1 := total thrust from all motors
%   w2 := thrust difference in c1 axis
%   w3 := thrust difference in c2 axis
%   w4 := imparted torque difference between motors on c1 axis and
%       motors on c2 axis


%% Parse input vectors

% Position
e1 = z(1);
e2 = z(2);
e3 = z(3);

% Attitude
phi = z(4);
theta = z(5);
psi = z(6);

% Velocity
e1_dot = z(7);
e2_dot = z(8);
e3_dot = z(9);

% Angular velocity
phi_dot = z(10);
theta_dot = z(11);
psi_dot = z(12);

% Desired position/velocity/acceleration
e1_d = z_d(1);
e2_d = z_d(2);
e3_d = z_d(3);
e1_dot_d = z_d(4);
e2_dot_d = z_d(5);
e3_dot_d = z_d(6);
e1_ddot_d = z_d(7);
e2_ddot_d = z_d(8);
e3_ddot_d = z_d(9);

% Desired angular velocity/accleration
phi_dot_d = 0;
theta_dot_d = 0;
psi_dot_d = 0;
phi_ddot_d = 0;
theta_ddot_d = 0;
psi_ddot_d = 0;

% Parameter vector
g = p(1);
l = p(2);
m = p(3);
I11 = p(4);
I22 = p(5);
I33 = p(6);
mu = p(7);
sigma = p(8);


%% Controller Constants

if control_mode == "capture"

    lambda1_e1 = 1.25;
    lambda2_e1 = 0.3;
    K_e1 = 0.05;
    
    lambda1_e2 = 1.25;
    lambda2_e2 = 0.3;
    K_e2 = 0.05;
    
    lambda_e3 = 1;
    K_e3 = 1;
    
    lambda_phi = 5;
    K_phi = 0.2;
    
    lambda_theta = 5;
    K_theta = 0.2;
    
    K_psi = 0;
    lambda_psi = 0;

elseif control_mode == "return"

    lambda1_e1 = 0.5;
    lambda2_e1 = 0.2;
    K_e1 = 0.1;
    
    lambda1_e2 = 0.5;
    lambda2_e2 = 0.2;
    K_e2 = 0.1;
    
    lambda_e3 = 2;
    K_e3 = 1;
    
    lambda_phi = 1.5;
    K_phi = 0.2;
    
    lambda_theta = 1.5;
    K_theta = 0.2;
    
    K_psi = 0;
    lambda_psi = 0;

end


%% Position Controller

% Sliding surface for altitude control
S_e1 = (e1_dot - e1_dot_d) + lambda1_e1*(e1 - e1_d);
S_e2 = (e2_dot - e2_dot_d) + lambda1_e2*(e2 - e2_d);
S_e3 = (e3_dot - e3_dot_d) + lambda_e3*(e3 - e3_d);

% Sliding mode altitude controller
w1 = (m/(cos(phi)*cos(theta))) * (e3_ddot_d + g - lambda_e3*(e3_dot - e3_dot_d) - K_e3*sign(S_e3));
wy = -(m/(w1*cos(psi)))*(e2_ddot_d - lambda1_e2*(e2_dot - e2_dot_d) - lambda2_e2*(e2 - e2_d) - K_e2*sign(S_e2)) + (cos(phi)*sin(psi)*sin(theta))/cos(psi);
wx = (m/(w1*cos(phi)*cos(psi)))*(e1_ddot_d - lambda1_e1*(e1_dot - e1_dot_d) - lambda2_e1*(e1 - e1_d) - K_e1*sign(S_e1)) - (sin(phi)*sin(psi))/(cos(phi)*cos(psi));

% Compute deisred roll and pitch angles
theta_d = asin(min(max(wx,-0.5),0.5));
phi_d = asin(min(max(wy,-0.5),0.5));
psi_d = 0;


%% Attitude Controller

% Sliding surfaces (attitude controller)
S_phi   = (phi_dot - phi_dot_d)     + lambda_phi*(phi - phi_d);
S_theta = (theta_dot - theta_dot_d) + lambda_theta*(theta - theta_d);
S_psi   = (psi_dot - psi_dot_d)     + lambda_psi*(psi - psi_d);

% Attitude controller
w2 = I11*(phi_ddot_d   - ((I22-I33)/I11)*theta_dot*psi_dot - lambda_phi*(phi_dot - phi_dot_d)       - K_phi*sign(S_phi));
w3 = I22*(theta_ddot_d - ((I33-I11)/I22)*phi_dot*psi_dot   - lambda_theta*(theta_dot - theta_dot_d) - K_theta*sign(S_theta));
w4 = I33*(psi_ddot_d   - ((I11-I22)/I33)*phi_dot*theta_dot - lambda_psi*(psi_dot - psi_dot_d)       - K_psi*sign(S_psi));


%% Control mixing block

% Map control inputs w1, w2, w3, w4 to motor speed inputs u1, u2, u3, u4
u1 = w1/4 - w3/(2*l) + w4/(4*sigma);
u2 = w1/4 + w2/(2*l) - w4/(4*sigma);
u3 = w1/4 + w3/(2*l) + w4/(4*sigma);
u4 = w1/4 - w2/(2*l) - w4/(4*sigma);

%% Construct control input vector

u = [u1 u2 u3 u4];

