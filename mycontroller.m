function [sys,x0,str,ts] = mycontroller(t,x,u,flag)
%CSFUNC An example MATLAB file S-function for defining a continuous system.  
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
% end csfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes

sizes = simsizes;
sizes.NumContStates  = 5;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 7;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = [10,10,10,10,10];
str = [];
ts  = [0 0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)
Tl     = 1; %Parameter Load Torque
M      = 0.17;
Rr     = 1.18;
Rs     = 1.34;
Lr     = 0.18;
Ls     = 0.18;
B      = 0.00014;
J      = 0.0153;
np     = 6;                 %3 or 6 %6 poles, 3 pole pairs
sigma  = 1-((M^2)/(Lr*Ls)); %unfound parameter 

eta    = Rr/Lr;
beta   = M/(sigma*Lr*Ls);
mu     = np*M/(J*Lr);
gamma  = ((M^2)*Rr/(sigma*Lr^2*Ls))+(Rs/(sigma*Ls));

a1     = mu;
a2     = Tl/J;
a3     = eta;
a4     = eta*M;
a5     = np;

xdot   =     [a1*x(2)*x(4)-a2;
             -a3*x(2)+a4*x(3);
              u(1);
              u(2);
              a5*x(1) + a4*(x(3)/x(2))];
sys = xdot;

% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
Tl     = 1; %Parameter Load Torque
M      = 0.17;
Rr     = 1.18;
Rs     = 1.34;
Lr     = 0.18;
Ls     = 0.18;
B      = 0.00014;
J      = 0.0153;
np     = 6;                 %3 or 6 %6 poles, 3 pole pairs
sigma  = 1-((M^2)/(Lr*Ls)); %unfound parameter 

eta    = Rr/Lr;
beta   = M/(sigma*Lr*Ls);
mu     = np*M/(J*Lr);
gamma  = ((M^2)*Rr/(sigma*Lr^2*Ls))+(Rs/(sigma*Ls));

% a1     = mu;
% a2     = Tl/J;
% a3     = eta;
% a4     = eta*M;
% a5     = np;

% Set Points
x1d = 1000;%u(3) %100; % speed reference
x2d =  0.05%;jn5;%u(4) %0;   % flux reference

a1 = mu;
a2 = Tl/J;
a3 = eta;
a4 = eta*M;
%a5 = np;

b1 = sigma*Ls;
b2 = gamma;
b3 = eta*beta;
b4 = np;
b5 = eta*M;
b6 = beta*np;

c1 = 1000;
c2 = 1000;
c3 = 1000;
c4 = 1000;

x1ddot =  0;
x2ddot =  0;

z1 = x2d  - x(2);
z2 = x(3) - (1/a4)*(((a3*x(2)) + x2ddot + c1*z1));
z3 = x1d  - x(1);
z4 = x(4) - ((1/(a1*x(2)))*(x1ddot + a2 + c3*z3)) ;

z1dot = x2ddot + a3*x(2) - a4*x(3);
z3dot = x1ddot - a1*x(2)*x(4) + a2;

x1d2dot = 0;
x2d2dot = 0;

%vd = (1/z1)*(a4*z1 + (a3/a4)*x2dot + x2d2dot + c1*z1dot - c2*z2) 
%vd = (a4*z1 + (a3/a4)*x2dot + x2d2dot + c1*z1dot - c2*z2); 
vd = (1/z2)*(a4*z1*z2 + (a3/a4)*(-a3*x(2)+a4*x(3)) + (x2d2dot/a4) + (c1*z1dot/a4) - c2*z2^2);
u1 = -b1*(-b2*x(3) + b3*x(2) + b4*x(1)*x(4) + b5*((x(4)^2)/x(2))) + b1*(vd)


%vq = ((a1*x(2)*x(3)) + a2*(z3/z4) + ((1/(a1*x2dot))*(x1d2dot + a2 + c3*z3dot)) - c4*z4)
%vq =(1/z2)*((a1*x(2)*x(3)) + a2*(z3/z4) + ((1/(a1*x2dot))*(x1d2dot + a2 + c3*z3dot)) - c4*z4);
vq = (1/z4)*(a1*x(2)*z3*z4) + (a2*z3) + ((z4/(-a1*(-a3*x(2)+a4*x(3))^2))*(x1d2dot + a2 + (c3*z3dot))) - c4*z4^2;
u2 = -b1*(-b2*x(4) - b6*x(1)*x(2) - b4*x(1)*x(3) - b5*((x(3)*x(4))/x(2))) + b1*(vq)

%sys = []
sys = [x(1); x(2); x(3); x(4); x(5); u1; u2];

% end mdlOutputs
