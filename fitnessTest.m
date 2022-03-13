%The point of this is to automate and find preferable controllers for any
%set of parameters for the bike, p, for any initial velocity, for any set
%of timelags. As such, it is a set of tests that parameterizes on p, uses a
%master controller [11.1, 2.4, -6.6] (still figuring out why that is a
%master controller, but hopefully many tests run on many different
%parameters will explain why). 

%Testing protocol: Input p, v0. Find: 1) best lqr controller, 2) 2 other
%random successful lqr controllers, 3)grid search controller. Then, test for 1)
%speed of stability, 2) max amplitude of delta_dot, 3) largest stabilizing
%actuation flaw against controllers 1,2,3 and [11, 2,-6] 

function [specs] = fitnessTest(p, v0)
delta0 = 0;
phi0 = pi/6;
phi_dot0 = 0;
x = 0; 
y = 10;
psi0 = 0;
v = v0; 

specs = zeros(5,8); 
specs(1,1:3) = ([11.1,2.4,-6.6]);
fprintf("Parameters: p.l ="+ p.l + "p.b  =" +p.b + " p.h = " +p.h); 
try
[K1,K2,K3]= findLQR(p, v0,0,0);
specs(2,1:3) = K1; 
specs(3,1:3) = K2;
specs(4,1:3) = K3; 
[K4] = findBC(p, v0, 0,0);
specs(5,1:3) = K4; 
t1 = 0.00;
t2 = 0.00;
figure("Name", "Parameters" +p.b+"," + p.h+ ","+ p.l);
hold off
subplot(2,2,1)
for a = 1:5
    [success,states, stable] = runBicycleTestR(x,y,v,delta0,phi0, phi_dot0,psi0,p,  specs(a,1:3), 0, t1,t2, 10000,  0);
    specs(a, 4) = stable; 
    
     plotController(x,y,v,delta0,phi0, ...
                phi_dot0,psi0,p,  specs(a,1:3), 0, t1,t2,  500,  0);
    [minspeed] = SpeedTest(specs(a,1:3), p);
    specs(a,5) = minspeed; 
   % fprintf("\n"+specs(a,1) + ", " + stable+"\n"); 
   

    [T1, T2,Tsum] = findOffset(specs(a,1:3), p, v0);
    specs(a,6) = T1;
    specs(a,7) = T2; 
    specs(a,8) = Tsum;
end
k_1 = specs(:,1);
k_2 = specs(:,2) ;
k_3 = specs(:,3 ); 
stabilize = specs(:,4); 
speed = specs(:,5);
tl1 = specs(:,6); 
tl2 = specs(:,7);
tls = specs(:,8);
T = table(k_1,k_2,k_3,stabilize, speed, tl1,tl2,tls)
legend("oldlqr","bestlqr","rand1","rand2","grid\_tuned");
hold off

catch 
    %global try/catch failure, to be refined. 
    fprintf("\nfailed\n")





end