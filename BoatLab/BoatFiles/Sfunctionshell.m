function [sys,x0,str,ts] = DiscKal(t,x,u,flag,data) %if method 2 is used
% Shell for the discrete kalman filter assignment in
% TTK4115 Linear Systems.
%
% Author: J?rgen Spj?tvold
% 19/10-2003 
%

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(data); %if method 2 is used

  %%%%%%%%%%%%%
  % Outputs   %
  %%%%%%%%%%%%%
  
  case 3,
    sys = mdlOutputs(t,x,u,data) %if mathod 2 is used
  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  
  case 2,
    sys = mdlUpdate(t,x,u, data); %if method 2 is used
  
  case {1,4,}
    sys=[];

  case 9,
      sys=mdlTerminate(t,x,u);
  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end

function [sys,x0,str,ts] = mdlInitializeSizes(data); %if method 2 is used
% This is called only at the start of the simulation. 

sizes = simsizes; % do not modify

sizes.NumContStates  = 0; % Number of continuous states in the system, do not modify
sizes.NumDiscStates  = 35; % Number of discrete states in the system, modify. (5x5)
sizes.NumOutputs     = 2; % Number of outputs, the hint states 2 (psi and b)
sizes.NumInputs      = 2; % Number of inputs, the hint states 2 (delta and measurement)
sizes.DirFeedthrough = 1; % 1 if the input is needed directly in the
% update part
sizes.NumSampleTimes = 1; % Do not modify  

sys = simsizes(sizes); % Do not modify  

x0  = [data.x0_h_', 0 0 0 0 0, mat2vec(data.P0_)']'; % Initial values for the discrete states, modify

str = []; % Do not modify

ts  = [-1 0]; % Sample time. [-1 0] means that sampling is
% inherited from the driving block and that it changes during
% minor steps.


function sys = mdlUpdate(t,x,u, data); %if method 2 is used
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update the filter covariance matrix and state etsimates here. %
% example: sys=x+u(1), means that the state vector after        %
% the update equals the previous state vector + input nr one.   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
P_= vec2mat(x(11:35));%Henter Apriori P av tilstandsvektoren
K = P_*data.Cd'*inv(data.Cd*P_*data.Cd'+data.R); %Beregner ny kalmanforsterkning
xh_= x(1:5);%Henter ut Apriori-estimatene fra tilstandsvektoren
xh = xh_ + K*(u(2)-data.Cd*xh_);%Beregner Aposteriori-estimatene P=(eye(5,5)-K*data.Cd)*P_;%Beregner Aposteriori feil-kovarians
xh_=data.Ad*xh + data.Bd*u(1);%Beregner aprioriestimat for tilstandene
P_= data.Ad*P_*data.Ad' + data.Ed*data.Q*data.Ed';%Beregner aprioriestimat for feil-kovarians
% skal v?re APA' og ikke AP_A', P = (I-L*C) * P_pri * (I - L*Cd)' + L * R * L';
% L = P_pri * Cd' * inv(Cd * P_pri * Cd' + R);

sys=[xh_', xh', mat2vec(P_)']';

function sys = mdlOutputs(t,x,u,data) %if mathod 2 is used
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate the outputs here                                        %
% example: sys=x(1)+u(2), means that the output is the first state+ %
% the second input.                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sys=[x(8), x(10)];

function sys = mdlTerminate(t,x,u) 
sys = [];


