function [zdot,u]=rhs_init(currentState,p,K,init_u, delta_offset, phi_offset,linear)
%see rhs.m for primary documentation; this is primarily an edit to allow
%for an initial but unintegrated control signal (motor commands that aren't
%changed at each timestep) and to allow for nonlinear EOM in the
%simulation. 

%unpack parameters
g=p.g; l=p.l; b=p.b; h=p.h;

%unpack gains
k1 = K(1); k2 = K(2); k3 = K(3);

%unpack state
x=currentState(2);
y=currentState(3);
phi=currentState(4);
psi=currentState(5);
delta=currentState(6);
phi_dot=currentState(7);
v = currentState(8);

%assign values to time derivatives of all initial conditions, use state
%variables (inputs) to define these. %that, as well as control output u, 
%is the function output;

%c3 should have a different sign from the other gains.
u=k1*(phi-phi_offset)+k2*phi_dot+k3*(delta-delta_offset)+init_u; %u, control variable, is delta dot

%set limit for maximum allowable steer rate
%max steer rate of 4.8 rad/s from ABt Fall '17 report, page 11
if u>4.8
    u=4.8; %rad/s         
elseif u<-4.8
    u=-4.8;
end

%calculate derivatives from equations of motion. 
%Shihao Wang's 2014 report has more explanation on EOM.
xdot=v*cos(psi);
ydot=v*sin(psi);
phi_dot=phi_dot;
psi_dot=(v/l)*(tan(delta)/cos(phi));
delta_dot=u;
v_dot=0;

phi_ddot = (1/h)*...
   (g*sin(phi) - tan(delta).*...
      (v.^2/l + b*v_dot/l + tan(phi).*...
           ((b/l)*v.*phi_dot - (h/l^2)*v.^2.*tan(delta)))...
   -b*v.*delta_dot./(l*cos(delta).^2)); %nonlinear EOM
%g/h -(v^2)/(h*l)-k2*phi_dot*b*v/(h*l*(cos(delta)^2));%+((v/l)^2*(tan(delta))^2*tan(phi));
%now we have all our terms in a vector representing the rhs
zdot=[xdot,ydot,phi_dot,psi_dot,delta_dot,phi_ddot,v_dot];