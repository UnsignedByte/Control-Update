function [T1, T2, Tsum] = findOffset(K,p,v0)
%examines the balancing capabilities for a range of observing and actuating
%offsets. Current minimum is set to 0.01 to better examine upper range of
%offsets, but ranges can be adjusted.
x =linspace(.2,0.00,40);
y1 = linspace(.2, 0.00,40);
result = zeros(length(x)*length(y1),4);
trial = 1;
for x=x
    for y =y1
   [success,state] = runBicycleTestR(0,0,v0,0,pi/6,0,0,p,K, 0,x,y,500,0); 
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

%T1 = successes(1,2); 
%successes1 = sortrows(successes, 3, 'descend'); 
%T2 = successes1(1,3); 
%successes2 = sortrows(successes, 4, 'descend'); 
%Tsum  = successes2(1,4); 
M = max(successes); 
if(isempty(M))
    T1 = -1;
    T2 = -1;
    Tsum = -1;
else
T1 = M(2); 
T2 = M(3); 
Tsum= M(4);
end 
%fprintf("best   " + T1+"best   " +T2 + "best   " +Tsum);
% figure('Name','balancing with two offsets between 0.01 and 0.1')
% subplot(2,2,1)
numtime = 200;
for i=1:20%length(successes)
%fprintf('lag 1 = %d   lag2=   %d   sum lag=   %d\n', successes(i,2), successes(i,3), successes(i,2)+successes(i,3));
    %display(successes(i,2));
%   plotController(0,0,v0,0,pi/6, ...
%                 0,0,p,  K,0, successes(i,2),successes(i,3),  300,  0);
%  

end
%figure('Name','graph of largest offset between observing and changing motor commands')
% subplot(2,2,1)
%     plotController(0,0,v0,0,pi/6, ...
%                 0,0,p,  K,0, successes(1,2),successes(1,3),  300,  0);
%             display(length(successes));
end