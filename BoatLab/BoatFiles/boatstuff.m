% Assorted code snippets for the TTK4115-Linear Systemstheory Boat assignement
%% Variable initialization
w0 = 0.7823;
T = 74.435;
K = 0.1707;
T_d = T;
T_f = 8.391;         
K_pd = 0.7647;       
phi = 30;
x = 0.0827;
sigma = sqrt(0.001484);
%
set(0,'DefaultTextInterpreter', 'latex')
plot(kalman_estimate); hold on; plot(kalman_estimate_wave); legend('Psi','Filtered psi')
%
%% Task 5.1 - Identification of the boat parameters
figure()
set(gcf,'color','w');
subplot(2,1,1); plot(noDist1); hold on; grid on;
                plot(waveNmeasure1);    
                title('System with rudder input of $\sin(0.005t)$')
           legend('No disturbances','Wave disturbance & measurement noise')
                xlabel('time [s]','Interpreter','latex');
                ylabel('amplitude','Interpreter','latex')
subplot(2,1,2); plot(noDist2); hold on; grid on;
                plot(waveNmeasure2);    
                title('System with rudder input of $\sin(0.05t)$')
           legend('No disturbances','Wave disturbance & measurement noise')
                xlabel('time [s]','Interpreter','latex')
                ylabel('amplitude','Interpreter','latex')
                
%% Task 5.1 d) - Transfer function
figure()
set(gcf,'color','w')
plot(transfer);     hold on;    grid on;
plot(ship);
title('Ship and transfer function response to constant rudder input.')
legend('Transfer function','Ship model')
xlabel('time [s]','Interpreter','latex')

%% Task 5.2 - Identification of wave spectrum model
[pxx,f] = pwelch(psi_w(2,:)*pi/180,4096,[],[],10);

pxx = pxx/(2*pi);       w = f*2*pi;    % Scaling

w0 = 0.7823;                           % Found from plot
sigma = sqrt(0.001484);                % --//--
L = 1;

P_psi_w_fun = @(L,w) (4.*L^2.*w0^2.*sigma^2.*w.^2)./(w.^4 + (4.*L^2-2).*w0^2*w.^2 + w0^4);

x = lsqcurvefit(P_psi_w_fun,L,w,pxx);  % Using curvefit to find best lambda

P_psi_w = (4.*x^2.*w0^2.*sigma^2.*w.^2)./(w.^4 + (4.*x^2-2).*w0^2*w.^2 + w0^4);

figure()
plot(w,pxx);         axis([0 1.8 0 0.0016]);    hold on;
plot(w,P_psi_w);     legend('PSD estimate','PSD(real), fitted')
title('Estimated vs Real PSD')
xlabel('w$[\frac{{rad}}{{s}}]$','Interpreter','latex');
ylabel('$[W\frac{s}{rad}]$','Interpreter','latex')

%% Task 5.3 - Control system design
T = 74.435;
K = 0.1707;
T_d = T;
T_f = 8.391;         %Found by comparing the real and complex values of 
K_pd = 0.7647;       % H_s(jw) with cos(phi) and sin(phi) respectively,
                     % with w = w_c = 0.1 and phi = 50 degrees.

H_pd = tf(K_pd*[T_d 1],[T_f 1]);
B = K_pd*[T_d 1]; A = [T_f 1];
x = randn(1000,1);
y = filter(B,A,x);
margin(H_pd)
%plot(x); hold on; plot(y); legend('unfiltered', 'filtered')
H_ship = tf([T_f*T T_f+T 1 0]);
H_s = tf(K_pd*K*[1],[T_f 1 0])
%figure()
set(gcf,'color','w')
margin(H_s); grid on;
%bode(feedback(H_s,1)) 

%% Task 5.3 b-d)
phi = 30;
figure()
set(gcf,'color','w');
% Compass plots
subplot(2,1,1); plot(compass_wave,'b');  grid on;    hold on;    
                title('Compass with ref. 30 deg.')
                plot(compass_current);      plot(compass,'r')
legend('Only measurement noise', 'Current disturbance', 'Wave disturbance')
title('Ship simulations with PD controller.','Interpreter','latex')
xlabel('Time [s]', 'Interpreter', 'latex'); 
ylabel('Angle [deg.]', 'Interpreter', 'latex')
axis([0,700,-5,35]);
% Rudder input plots
subplot(2,1,2); plot(rudder,'r');   grid on;    hold on;    title('Rudder')
                plot(rudder_current);       plot(rudder_wave,'b');
legend('Only measurement noise', 'Current disturbance', 'Wave disturbance');
title('Rudder input.','Interpreter','latex')
xlabel('Time [s]','Interpreter','latex');     
ylabel('Angle [deg.]','Interpreter','latex');     axis([0,600,-40,50]);
 
%% Task 5.4 - Discrete State-space Model
A = [0 1 0 0 0;
     -w0^2 -2*x*w0 0 0 0;
     0 0 0 1 0;
     0 0 0 -1/T -K/T;
     0 0 0 0 0];
 B = [0 0 0 K/T 0]';
 E = [0 2*x*w0*sigma 0 0 0;
      0 0 0 0 1]';
 C = [0 1 1 0 0];
 D = 1;
 
 [Ad, Bd] = c2d(A,B,0.1);
 [Ad, Ed] = c2d(A,E,0.1);
 Cd = C;
 Dd = D;
 
 Q = [30 0;
      0 10^(-6)];
 P0_ = [1 0 0 0 0;
        0 0.013 0 0 0;
        0 0 pi^2 0 0;
        0 0 0 1 0;
        0 0 0 0 2.5*10^(-4)];
 x0h_ = [0 0 0 0 0]';
 R = 0.002/0.1;
 P0 = P0_;
 I = eye(5);
 model_struct = struct('Ad',Ad, 'Bd',Bd, 'Ed',Ed, 'Cd',Cd, 'Q',Q, 'R',R, 'P0_',P0_, 'x0_h_',x0h_, 'I',I);
 
 %% Test!!
 x = 1:1000:1; y = 90;
 figure()
 plot(kalman_estimate,'LineWidth', 1); hold on; 
 plot(compass,'LineWidth', 1); %hold on; plot(kalman_estimate_wavefiltered,'LineWidth', 1);  
 %legend('Compass angle w. PD (no Kalman filter)','Kalman estimate w, bias feedforward','Interpreter','latex');
 title('Compass heading vs Kalman filter estimate','Interpreter','latex');
 xlabel('Time','Interpreter','latex'); %axis([0,500,-1,35])
 line([1 1000],[90 90])
% 
%  figure()
%  plot(kalman_estimate); hold on;
%  plot(kalman_estimate2);
%  plot(kalman_estimate3);
%  plot(kalman_estimate4);
%  legend('Measure','Meas. + current','Meas. + wave','Meas. + current + wave')
%  plot(measurement_noise); plot(current_noise); plot(wave_noise); plot(NOISE)
% figure(); plot(kalman_bias); hold on; %plot(kalman_rudder)
% 
% figure()
% subplot(1,2,1); plot(kalman_rudder,'LineWidth', 1); 
% title('Rudder input ($\delta$)','Interpreter','latex');
% xlabel('Time','Interpreter','latex'); axis([0,200,-25,47])
% subplot(1,2,2);plot(kalman_bias,'LineWidth', 1);
% xlabel('Time','Interpreter','latex');
% title('Kalman filter - bias','Interpreter','latex'); %axis([0,500,0,3.5])
% 
% figure()
% subplot(2,1,1);
% plot(estimated_wave); hold on;
% plot(psi_w(2,1:10000))
% legend('Estimated wave influence','Actual wave influence','Interpreter','latex')
% title('System wave influences (actual vs estimated)','Interpreter','latex')
% subplot(2,1,2);
% plot(estimated_wave); hold on;
% plot(psi_w(2,1:10000));
% legend('Estimated wave influence','Actual wave influence','Interpreter','latex')
% title('System wave influences (actual vs estimated)','Interpreter','latex')
% axis([0,1200,-3.2,3.2])

% figure()
% plot(Noise); hold on;    plot(kalman_estimate);
% plot(wave_noise); %plot(current_noise)
% axis([0,8,-.5,3])
% legend('Sum of discturbances','Kalman estimate','Wave disturbance','Interpreter','latex')
% title('Initial Kalman estimate vs disturbances','Interpreter','latex')
% xlabel('Time')