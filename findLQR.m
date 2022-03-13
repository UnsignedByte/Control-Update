function [K1, K2, K3] = findLQR(p,v0, lag1,lag2)

%findLQR is modified from the file in Matlab-Optimization-master
%lqr_BC_optimizer.m, and operates very similarly. It is modified in that it
%takes in arguments for the bicycle parameters (p), initial velocity (v0)
%and two different delays in operation (see runBicycleTestR/findOffset). 
%operates by tuning several different proportions of a gain matrix,
%computing a controller with the LQR algorithm, and then selecting 3
%successful controllers: K1 is the controller with the best balance score,
%and K2, K3 are both randomly selected for comparison. 

%Initial Conditions
delta0 = 0;
phi0 = pi/6;
phi_dot0 = 0;
x = 0; 
y = 10;
psi0 = 0;
v = v0;  %m/s

%%%%%%%%%%    LQR %%%%%%%%%%%%%%
% constant matricies from Shihao Wang's 2014 Report
    %define parameters  
    % !!!!!!!!!    THESE ARE DUPLICATED IN "runBicycleTest.m"    !!!!!!!!!
       g = p.g; %acceleration due to gravity
       l = p.l; %length of wheel base 
       b =  p.b; %distance from rear wheel to COM projected onto ground
       h = p.h; %height of COM in point mass model
        % h is not the same as the height of COM of the bicycle, h is
        % calculated to place the center of mass so that the point
        % mass model and the real bicycle fall with the same falling frequency.
        % see ABT Fall 2017 report for further discussion.
       c = 0;   %trail

A = [   0       1       0
     g/h      0  -(v^2)/(h*l)
       0       0       0     ];
B = [   0  -b*v/(h*l)   1]';

%t = linspace(0.01,1,100);
t = logspace(-1,4,100);
result = zeros(length(t),8);
trial = 1;

for t = t

    Q = t*[1 0 0; 0 0.1 0; 0 0 0];
    R = [1];

    [K,S,e] = lqr(A,B,Q,R);
    K = -1*K; %get K from lqr controller to match sign convention


    [success, state] = runBicycleTestR(x,y,v,delta0,phi0,phi_dot0,psi0,p,K,0,lag1, lag2,400, 0);  
    phi = state(:,4);
    delta = state(:,6);
    phidot = state(:,7);
    psi = state(:,5);
    xb = state(:,2);
    yb = state(:,3);
    
    

    % Was run Successful?
    result(trial,1) = success;
    
    %Scoring for Balance (want lean rate to converge to 0)
    result(trial,2) = sqrt(sum(phi.^2)+sum(phidot.^2)/4+sum(delta.^2));
    %result(trial,2) = sqrt(sum(phi.^2));

    result(trial,4) = K(1);
    result(trial,5) = K(2);
    result(trial,6) = K(3);
    result(trial,7) = t;
    result(trial,8) = e(1);
    result(trial,9) = e(2);
    result(trial,10) =e(3);
    trial = trial + 1;
    
end
    

success = result(:,1);
balance_score = result(:,2);
path_score = result(:,3);
k_1 = result(:,4);
k_2 = result(:,5);
k_3 = result(:,6);
ratio = result(:,7);
e1 = result(:,8);
e2 = result(:,9);
e3 = result(:,10);

T = table(success,balance_score,path_score,k_1,k_2,k_3,ratio,e1,e2,e3);
m = table2array(T);

%Find best test based on balance score:
m = sortrows(m,2);
indm = find(m(:,1));  %filters out failures
%best test from balance score:
best1 = m(indm(1),:); 
%two randomly successful cases. Note: this allows for a small chance of
%duplicates in the case that there are not many successful trial runs.
rand1 = cast(rand()*length(indm), 'uint8');
rand2 = cast(rand()*length(indm), 'uint8'); 
alt1 = m(indm(rand1),:);
alt2 = m(indm(rand2),:);
K1 =[best1(4),best1(5),best1(6)];
K2 = [alt1(4), alt1(5), alt1(6)];
K3 = [alt2(4), alt2(5), alt2(6)];

end