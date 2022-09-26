p.g = 9.81; 
p.l = 1.02;
p.b = 0.33; 
p.h = 0.516; 
v0 = 3; 
delta_offset = zeros(1,500); 
delta_offset(1,1:100) = ones(1,100)*0.3;
delta_offset(1, 301:370) = ones(1, 70)*-0.1;
%delta_offset(1, 501:600) = ones(1, 100)*-0.3;
%delta_offset(1, 801:900) = ones(1, 100)*0.5;
x0 = 0; 
y0 = 0;
delta0 = 0;
phi0 = pi/12; 
phi_dot0 = 0; 
psi0 = 0;
lag1 = 0.00; 
lag2 = 0.01;
nonlinear = 1; 
a = 0.3;
plot = 0; 
size = 5000; 
seedings = 3; 
param = zeros(2, 6);
param(1, :) = [-100, -40, 50, -20, -20, -50];
param(2,:) = [200, 70, -100, 50, 40, 60];

[KB, KM] = find_nonlinear(p, v0, delta_offset,x0, y0, delta0,phi0, phi_dot0, psi0,lag1, lag2, nonlinear, a, plot, size, seedings, param, 0.8)
[K, K1, K2] = findLQR(p,v0, lag1, lag2)
plotController(x0,y0,v0,delta0,phi0, ...
                phi_dot0,psi0,p,  K, delta_offset, lag1,lag2,  500,  0)
testSteerOffset(p, KB, v0, delta_offset,x0, y0, delta0,phi0, phi_dot0, psi0,lag1, lag2, nonlinear, a,KM, 1);