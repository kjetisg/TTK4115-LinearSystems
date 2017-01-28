% This file contains the initialization for the helicopter assignment in
% the course TTK4115. Run this file before you execute QuaRC_ -> Build 
% to build the file heli_q8.mdl.

% Oppdatert høsten 2006 av Jostein Bakkeheim
% Oppdatert høsten 2008 av Arnfinn Aas Eielsen
% Oppdatert høsten 2009 av Jonathan Ronen
% Updated fall 2010, Dominik Breu
% Updated fall 2013, Mark Haring
% Updated spring 2015, Mark Haring


%%%%%%%%%%% Calibration of the encoder and the hardware for the specific
%%%%%%%%%%% helicopter
Joystick_gain_x = 1;
Joystick_gain_y = -1;


%%%%%%%%%%% Physical constants
g = 9.81; % gravitational constant [m/s^2]
l_c = 0.46; % distance elevation axis to counterweight [m]
l_h = 0.66; % distance elevation axis to helicopter head [m]
l_p = 0.175; % distance pitch axis to motor [m]
m_c = 1.92; % Counterweight mass [kg]
m_p = 0.72; % Motor mass [kg]
Vs = 6.422; % Motor voltage equalibrium
K_f = -(g*(l_c*m_c-2*l_h*m_p))/(l_h*Vs); %Motor force constant
J_p= (2*m_p*l_p^2); %Moment of inertia pitch
J_lambda=m_c*l_c^2+2*m_p*l_h^2; %Moment of inertia travel
J_e = m_c * l_c^2 + 2*m_p*l_h^2; % Moment of inertia elevation
L_1 = l_p * K_f;
L_2 = g*(l_c*m_c-2*l_h*m_p);
L_3 = l_h*K_f*Vs;
L_4 = -l_h*K_f;
K_1 = L_1/J_p;
K_2 = L_3 / J_e;
K_3 = L_4 / J_lambda;

%%%%%%%%%%% PD-values, pitch controller

K_pp = 2; 
K_pd = 2*sqrt(K_pp*K_1)/K_1   % Dette blir vel riktig?
%K_pd = 2/sqrt(K_1);
Kjetil = 4/K_1;
%K_pd=sqrt(Kjetil)*K_pp;
%K_pd=(2/K_1)*sqrt(K_1*K_pp);

%%%%%%%%%%% P-values, travel controller
K_rp = 15;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 5.3.1-2
A_o3 = [0,1,0,;
     0,0,0,;
     0,0,0]
     
B_o3 =  [0,0;
      0,K_1;
      K_2,0]
C_o3 = [1,0,0; 0,0,1];
D_o3 = zeros(2,2);
Q_o3 = diag([15,1,10]);
R_o3 = diag([.1,.3]);
K_o3=lqr(A_o3, B_o3, Q_o3, R_o3)
P_o3 = inv(C_o3*inv(B_o3*K_o3-A_o3)*B_o3)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

A = [0,1,0,0,0;
     0,0,0,0,0;
     0,0,0,0,0;
     1,0,0,0,0;
     0,0,1,0,0]
B =  [0,0;
      0,K_1;
      K_2,0;
      0,0;
      0,0];
C = eye(5);
D = zeros(2,2);


Q = diag([15,1,10,10,20]);
R = diag([.1,.3]);

K=lqr(A, B, Q, R)

%P = inv(C*inv(B*K-A)*B);
P=[K(1,1), K(1,5); K(2,1),K(2,5)]

%% Estimering
%PI%
A_est = [0,1,0,0,0,0;
        0,0,0,0,0,0;
        0,0,0,1,0,0;
        0,0,0,0,0,0;
        0,0,0,0,0,1;
        K_3,0,0,0,0,0];
B_est = [0,0,0,K_2,0,0;
        0,K_1,0,0,0,0]';
C_est = [1,0,0,0,0,0;
        0,0,1,0,0,0;
        0,0,0,0,1,0];
poler = 8*[-15.3+36.95*i, -15.3-36.95*i, -28.28 + 28.28*i, -28.28 - 28.28*i, -36.95+15.3*i, -36.95-15.3*i];
EIGpi=eig(A-B*K)
L = place(A_est',C_est',poler)'
%L = [-1,-2,-3]
Q_uest = diag([15,1,10,2,15]);
R = diag([.1,.3]);
CO = ctrb(A_est,B_est);

%P%
%Ppoler = [-30+10*i -30-10*i -16+20*i -16-20*i -6+30*i -6-30*i]
Ppoler = [-15.3+36.95*i, -15.3-36.95*i, -28.28 + 28.28*i, -28.28 - 28.28*i, -36.95+15.3*i, -36.95-15.3*i]
L_p = place(A_est',C_est',Ppoler)'
% plot(x_hat); hold on
% figure(2)
% plot(x_uten_hatt)
% 


%%
C_est2= [1,0,0,0,0,0;
        0,0,1,0,0,0];
rank(obsv(A_est,C_est2))
Ppoler = Ppoler * 0.2
L = place(A_est',C_est2',Ppoler)'

subplot(3,1,1)
plot(p); hold on; plot(p_est,'r')
title('Pitch vs Estimated pitch')
xlabel('Time')
ylabel('Angle, rad')

subplot(3,1,2)
plot(e); hold on; plot(e_est,'r')
title('Elevation vs Estimated elevation')
xlabel('Time')
ylabel('Angle, rad')

subplot(3,1,3)
plot(p_dot); hold on; plot(p_dot_est,'r')
title('Pitchrate')
xlabel('Time')
ylabel('Angle, rad')
%
%
figure()
subplot(3,1,1); plot(e_dot); hold on; plot(e_dot_est,'r');
title('Elev.rate vs Estimated elev.rate')

subplot(3,1,2); plot(lambda); hold on; plot(lambda_est,'r');
title('Travel vs Estimated travel')

subplot(3,1,3); plot(lambda_dot); hold on; plot(lambda_dot_est,'r');
title('Travel rate vs Estimated travel rate')

    
