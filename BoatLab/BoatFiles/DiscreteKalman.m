% function [ x_est_post ] = DiscreteKalman( u, model )
% %DISCRETEKALMAN Summary of this function goes here
% %   Detailed explanation goes here
% struct_names = fieldnames(model);
% Ad = model.(struct_names{1});
% Bd = model.(struct_names{2});
% Ed = model.(struct_names{3});
% Cd = model.(struct_names{4});
% Q = model.(struct_names{5});
% R = model.(struct_names{6});
% P0 = model.(struct_names{7});
% x0 = model.(struct_names{8});
% P_pri = 0;
% I = eye(5);
% 
% persistent init_flag 
% 
% if isempty(init_flag)
%     init_flag = 1;
%     x_0_est_pri = x0;
%     P_pri = P0;
%     x_est_pri = x_0_est_pri;
% end
% 
% y = Cd*x_0_est_pri;
% % Kalman gain
% L = P_pri * Cd' * inv(Cd * P_pri * Cd' + R);
% % Update estimate
% x_est_post = x_est_pri + L * (y - Cd * x_est_pri);
% % Update error cov. matrix
% P = (I - L*Cd) * P_pri * (I - L*Cd)' + L * R * L';
% 
% x_est_next_pri = Ad * x_est_post + Bd*u;
% P_next_pri = Ad*P*Ad' + Ed*Q*Ed';
% 
% x_est_pri = x_est_next_pri;
% P_pri = P_next_pri;
% 
% output = x_est_post;
% end
% 

function output = DiscreteKalman( y, u, model_struct) %,Ad,Bd, Cd, Ed, R, P0,Q)
%DISCRETEKALMAN Summary of this function goes here
%   Detailed explanation goes here

persistent init_flag P_pri x_est_pri
% struct_names = fieldnames(model);
% Ad = model.(struct_names{1});
% Bd = model.(struct_names{2});
% Ed = model.(struct_names{3});
% Cd = model.(struct_names{4});
% Q = model.(struct_names{5});
% R = model.(struct_names{6});
% P0 = model.(struct_names{7});
% x0 = model.(struct_names{8});
%P_pri = 0;
Ad = model_struct.Ad;
Bd = model_struct.Bd;
Ed = model_struct.Ed;
Cd = model_struct.Cd;
Q = model_struct.Q;
R = model_struct.R;
I = model_struct.I;

if isempty(init_flag)
    init_flag = 1;
    x_0_est_pri = [0;0;0;0;0];
    P_0_pri = [1 0 0 0 0;
             0 0.13 0 0 0;
             0 0 pi^2 0 0;
             0 0 0 1 0;
             0 0 0 0 2.5*10^(-4)];
    P_pri = P_0_pri;
    x_est_pri = x_0_est_pri;
end

%y = Cd*x_est_pri;
% Kalman gain
L = P_pri * Cd' * inv(Cd * P_pri * Cd' + R);

% Update estimate
x_est_post = x_est_pri + L * (y - Cd * x_est_pri);

% Update error cov. matrix
P = (I - L*Cd) * P_pri * (I - L*Cd)' + L * R * L';

x_est_next_pri = Ad * x_est_post + Bd*u;
P_next_pri = Ad*P*Ad' + Ed*Q*Ed';

x_est_pri = x_est_next_pri;
P_pri = P_next_pri;

output = [x_est_post];
end

