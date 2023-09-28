function [K1] = findBC(p,v0, lag1,lag2)
delta0 = 0;
phi0 = pi/6;
phi_dot0 = 0;
x = 0; 
y = 10;
psi0 = 0;
v = v0;  %m/s

%k3 should be the opposite sign of k1 and k2. GAINS
k1 = [1:3];
k2 = [0:10];
k3 = [-15:-5];  

result = zeros(length(k1)*length(k2)*length(k3),5);
trial = 1; 

while(true)
for a=k1
    for b=k2
        for c=k3
            K = [a, b, c];

            [success, state] = runBicycleTestR(x,y,v,delta0,phi0,phi_dot0,psi0,p,K,0,lag1,lag2, 400,0,0);  
            
            phi = state(:,4);
            delta = state(:,6);
            phidot = state(:,7);
            psi = state(:,5);
            xb = state(:,2);
            yb = state(:,3);
            
            % Was run Successful?
            result(trial,1) = success;
            
            %Scoring for Balance (want lean rate to converge to 0)
            %result(trial,2) = sqrt(sum(phidot.^2)+sum(phi.^2)+sum(delta.^2));
            result(trial,2) = sqrt(sum(phi.^2)+sum(phidot.^2)+sum(delta.^2));
            
%             %Scoring for Path location by distance from actual final waypoint
%             result(trial,3) = sqrt((x(end) - xb(end))^2+(y(end)-yb(end))^2);
            
            %Scoring for Path location based on yaw rate
            result(trial, 3) = 0;
            
            result(trial,4) = a;
            result(trial,5) = b;
            result(trial,6) = c;
            trial = trial + 1;
        end
    end
end

success = result(:,1);
balance_score = result(:,2);
path_score = result(:,3);
k_1 = result(:,4);
k_2 = result(:,5);
k_3 = result(:,6);

T = table(success,balance_score,path_score,k_1,k_2,k_3);
m = table2array(T);

%Find best test based on balance score:
m = sortrows(m,2);
indm = find(m(:,1));  %filters out failures
if(length(indm)==0)
 k1 = [k1(length(k1))-1:k1(length(k1))+1];
 continue;
end 
best1 = m(indm(1),:); 
%fprintf('k1 = %d\nk2 = %d\nk3 = %d\n',best1(4),best1(5),best1(6))

K = [best1(4), best1(5), best1(6)];

if(best1(4)==k1(1))
    k1 = [k1(1) - 1:k1(1)+1];
    continue;
end
if (best1(4)>=k1(length(k1))||length(best1)==0) 
    k1 = [k1(length(k1))-1:k1(length(k1))+1];
    continue;
end
if(best1(5) == k2(1)) 
    k2 = [(k2(1) - 1):(k2(1)+1)];
    continue;
end
if (best1(5)==k2(length(k2))) 
    k2 = [k2(length(k2))-1:k2(length(k2))+1];
    continue;
end
if(best1(6) == k3(1)) 
    k3 = [k3(1) - 1:k3(1)+1];
    continue;
end
if (best1(6)==k3(length(k3))) 
    k3 = [k3(length(k3))-1:k3(length(k3))+1];
    continue;
end
if(best1(4)~= k1(1) &&best1(4)~= k1(length(k1))&&best1(5)~=k2(1)&&best1(5)~=k2(length(k2))&&best1(6)~=k3(1)&&best1(6)~=k3(length(k3)))
    break;
end
end
%fprintf("done!");
K1 = [K(1),K(2),K(3)];
end 





