function[XDOT]=B2_6DOF_model(X,U)

% extract the state vector
x1=X(1);      % Velocity in body x axis
x2=X(2);      % Velocity in body y axis
x3=X(3);      % Velocity in body z axis
x4=X(4);      % angular rate in body x axis
x5=X(5);      % angular rate in body y axis
x6=X(6);      % angular rate in body z axis
x7=X(7);      % phi (Bank angle)
x8=X(8);      % theta (Pitch angle)
x9=X(9);      % phi (Yaw angle)

% extract the control vector
u1=U(1);      %d_OE (outboard elevons)
u2=U(2);      %d_IE (inboard+mid elevons)
u3=U(3);      %d_R (split rudders)
u4=U(4);      %d_th1 (throttle 1)
u5=U(5);      %d_th2 (throttle 2)

% Nominal vehicle constants
m=152633.83;                   % Aircraft total mass
% NOTE:- We will define the Ib and invIb later
cbar=12.03;                  % Mean aerodynamic chord
lt=00;                   % Distance by AC of tail and body (m)
S=475;                     % Wing planform area (m^2)
St=00;                     % Tail planform area (m^2)
Xcg=0.84*cbar;             % X Position of cg in Fm (m)
Ycg=0;                     % Y Position of cg in Fm (m)
Zcg=0*cbar;             % Z Position of cg in Fm (m)
Xac=0.25*cbar;             % X position of aerodynamic center in Fm (m)
Yac=0;                     % Y position of aerodynamic center in Fm (m)
Zac=0;                     % Z position of aerodynamic center in Fm (m)
% Engine Constants
Xapt1=8;                   % X position of engine 1 force in Fm (m)
Yapt1=-5;               % Y position of engine 1 force in Fm (m)
Zapt1=-1.5;                % Z position of engine 1 force in Fm (m)
Xapt2=8;                   % X position of engine 2 force in Fm (m)
Yapt2=5;                % Y position of engine 2 force in Fm (m)
Zapt2=-1.5;                % Z position of engine 2 force in Fm (m)
% Other constants
rho=1.168;                 % Air density averaged for 0 to 1 km in (kg/m^3)
g=9.81;                    % Acceleration due to gravity (m/s^2)
depsda=0.1;               % Change in downwash w.r.t alpha (rad/rad)
alpha_L0=-0.058;     % Zero lift angle of attack (rad)
n=5;                     % Slope of linear region of lift slope
a3=-768.5;                 % coefficient of alpha^3
a2=609.2;                  % coefficient of alpha^2
a1=-155.2;                 % coefficient of alpha^1
a0=-15.212;                % coefficient of alpha^0 (Different from RCAM Model)
alpha_switch=14*(pi/180); % alpha value lift slope goes from linear to non-linear

% Note that these can alternatively be enforced in simulink
 u1min=-15*(pi/180);
 u1max=25*(pi/180);
 
 u2min=-25*(pi/180);
 u2max=10*(pi/180);
 
 u3min=-30*(pi/180);
 u3max=30*(pi/180);
 
 u4min=0.5*(pi/180);
 u4max=10*(pi/180);
 
 u5min=0.5*(pi/180);
 u5max=10*(pi/180);
 
 if(u1>u1max)
     u1=u1max;
 elseif(u1<u1min)
     u1=u1min;
 end
 
 if(u2>u2max)
     u2=u2max;
 elseif(u2<u2min)
     u2=u2min;
 end
 
 if(u3>u3max)
     u3=u3max;
 elseif(u3<u3min)
     u3=u3min;
 end
 
 if(u4>u4max)
     u4=u1max;
 elseif(u4<u4min)
     u4=u4min;
 end
 
 if(u5>u5max)
     u5=u5max;
 elseif(u5<u5min)
     u5=u5min;
 end

% calculate the airspeed
Va=sqrt(x1^2+x2^2+x3^2);
%claculate the alph and beta
alpha=atan2(x3,x1);
beta=asin(x2/Va);
%claculation of the dynamic pressure
Q=0.5*rho*Va^2;
%Also define the vectors wbe_b and V_b
wbe_b=[x4;x5;x6];
V_b=[x1;x2;x3];

% Calculate the CL_wb
if alpha<=alpha_switch
      CL_wb=n*(alpha-alpha_L0);
else
      CL_wb=a3*alpha^3+a2*alpha^2+a1*alpha+a0;
end
% claculation of the Total lift force
CL=CL_wb;
% Total drag force (Neglecting the tail)
CD=0.13+0.07*(5.5*alpha+0.654)^2;
% Calculate the side force
CY=-1.6*beta+0.24*u3;

% calculate the actual dimensional forces. These are in F_s (Stability axis)
FA_s=[-CD*Q*S;CY*Q*S;-CL*Q*S];

% Rotate these forces to F_b (Body axis)
C_bs=[cos(alpha) 0 -sin(alpha);0 1 0;sin(alpha) 0 cos(alpha)];
FA_b=C_bs*FA_s;

%Calculate the moments in Fb.  Define eta,dCMdx and dCMdu
eta11=-1.4*beta;
eta21=-0.17;
eta31=(1-alpha*(180/(15*pi)))*beta;
eta=[eta11;eta21;eta31];
dCMdx=(cbar/Va)*[-0.75 0 -0.03;   % more acuurate and reliable data needed
                  0 -10 0;
                 -0.3 0 -0.1];
dCMdu1=[-0.6 -0.4 0.01;
       -0.4 -0.6 0;
       0 0 -0.03];
dCMdu2=[-0.6 0 0;
       0 -0.6 0;
       0 0 -0.03];
% Now calculate CM=[cl;cm;cn] about aerodynamic center in Fb
%if u1 == 0 || u2 == 0
%    CMac_b=eta+dCMdx*wbe_b+dCMdu2*[u1;u2;u3];
%else
%    CMac_b=eta+dCMdx*wbe_b+dCMdu1*[u1;u2;u3];
%end
CMac_b=eta+dCMdx*wbe_b+dCMdu2*[u1;u2;u3];
%Normalize to an aerodynamic moments
MAac_b=CMac_b*Q*S*cbar;

%Transfer moment to CG
rcg_b=[Xcg;Ycg;Zcg];
rac_b=[Xac;Yac;Zac];
MAcg_b=MAac_b+cross(FA_b,rcg_b-rac_b);

% normalising u3 for airbrake
u3_norm = abs(u3 / 30);

% First, calculate the thrust of each engine
F1=2*u4*m*g - 0.05*u3_norm;
F2=2*u5*m*g - 0.05*u3_norm;
%Assuming that the engine thrust is aligned with the Fb, we have
FE1_b=[F1;0;0];
FE2_b=[F2;0;0];
FE_b=FE1_b+FE2_b;
%Now the engine moment due to offset of engine thrust from cog
mew1=[Xcg-Xapt1;Yapt1-Ycg;Zcg-Zapt1];
mew2=[Xcg-Xapt2;Yapt2-Ycg;Zcg-Zapt2];
MEcg1_b=cross(mew1,FE1_b);
MEcg2_b=cross(mew2,FE2_b);
MEcg_b=MEcg1_b+MEcg2_b;

% Calculate the gravitational forces in the body frame. This causes no moments abot the CG
g_b=[-g*sin(x8);g*cos(x8)*sin(x7);g*cos(x8)*cos(x7)];
Fg_b=m*g_b;

%Inertia matrix
Ib=m*[40.07 0 -2.0923;0 64 0;-2.0923 0 99.92];
% Inverse of the inertia matrix
invIb=(1/m)*[0.0249836 0 0.000523151;0 0.015625 0;0.000523215 0 0.010019];
% From F_b ( all the forces in Fb ) and calculate the udot, vdot,wdot
F_b=Fg_b+FE_b+FA_b;
x1to3dot=(1/m)*F_b-cross(wbe_b,V_b);
%From Mcg_b (all the moments about the cog in Fb) and calculate pdot,qdpt,rdoot
Mcg_b=MAcg_b+MEcg_b;
x4to6dot=invIb*(Mcg_b-cross(wbe_b,Ib*wbe_b));
% Calculation of the phidot,thetadot,psidot
H_pi=[1 sin(x7)*tan(x8) cos(x7)*tan(x8);0 cos(x7) -sin(x7);0 sin(x7)/cos(x8) cos(x7)/cos(x8)];
x7to9dot=H_pi*wbe_b;

% Rotational Matrix from the 1 frame to 2 frame
C1v=[cos(x9) sin(x9) 0;
    -sin(x9) cos(x9) 0;
    0 0 1];
% Rotational Matrix from the 1 frame to 2 frame
C21=[cos(x8) 0 -sin(x8);
    0 1 0;
    sin(x8) 0 cos(x8)];
% Rotational Matrix from the 1 frame to 2 frame
Cb2=[1 0 0;
    0 sin(x7) cos(x7);
    0 -sin(x7) cos(x7)];
% Implimenting the Navigational Equation
Cbv=Cb2*C21*C1v;
Cvb=Cbv';
x10to12dot=Cvb*V_b;

XDOT=[x1to3dot;
    x4to6dot;
    x7to9dot;
    x10to12dot;
    ];
