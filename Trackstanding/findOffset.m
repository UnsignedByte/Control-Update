function [T1, T2, Tsum, Tmul] = findOffset(K,p,v0,nonlinear,a, Km)
%examines the balancing capabilities for a range of observing and actuating
%offsets. Current minimum is set to 0.01 to better examine upper range of
%offsets, but ranges can be adjusted.
x =linspace(0.00,0.090,15);
y1 = linspace(0.00, 0.090,15);
result = zeros(length(x)*length(y1),4);
trial = 1;
figure("Name", "trackstanding")

for x=x
    for y =y1
   [success,~,~] =  runBicycleTestR(0,0,v0,0,0.00, ...
                0,0,p, K, 0, x,y, 200,  0); 
           plotController(0,0,v0,0,0.00, ...
                0,0,p, K, 0, x,y, 200,  0, 0); 
   result(trial, 5) = y*x;
     result(trial,4) = y+x;
   result(trial,3) = y; 
   result(trial,2) = x;
   result(trial,1) = success;
   trial =trial+1;
    end
end
result = sortrows(result,1);

indexr = find(result(:, 1));
successes = result(indexr(:),:);
successes
%T1 = successes(1,2); 
%successes1 = sortrows(successes, 3, 'descend'); 
%T2 = successes1(1,3); 
%successes2 = sortrows(successes, 4, 'descend'); 
%Tsum  = successes2(1,4); 
%M = max(successes); 
if(isempty(successes))
    T1 = -1;
    T2 = -1;
    Tsum = -1;
    Tmul = -1;
else
res1 = sortrows(successes, 2, 'descend'); 
T1 = res1(1, 2:4); 
res2 = sortrows(successes,3, 'descend');
T2 = res2(1, 2:4); 

ress = sortrows(successes,4, 'descend');
Tsum= ress(1, (2:4));
resm = sortrows(successes,5, 'descend');
Tmul= resm(1, (2:4));
end  
numtime = 200;
for i=1:20%length(successes)
 

end
%figure('Name','graph of largest offset between observing and changing motor commands')
% subplot(2,2,1)
%     plotController(0,0,v0,0,pi/6, ...
%                 0,0,p,  K,0, successes(1,2),successes(1,3),  300,  0);
%             display(length(successes));
end