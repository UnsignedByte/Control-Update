function[minSpeed] = SpeedTest(K, p)
%SpeedTest.m - Tests the slowest speed at which the bike stabilizes for a
%given set of parameters p, on a given controller. Composes an array of
%successful trials sorted in ascending order of speed and descending order
%of time lags, returns minSpeed, the lowest successful speed overall. 
%test on 100-length vector of speeds below 3.
v = linspace(1,3.3, 100); 

%test with several evenly spaced reaction lags. (ordered so a stable sort
%will place larger offsets at earlier indices). 
lag1 = linspace(0.1,0, 4); 
lag2 = linspace(0.1,0,4);
%store speeds and offsets in a multi-column array
result = zeros(length(v)*length(lag1)*length(lag2),4);
trial =1;

%run simulations on every combination, note that this runs in cubic time,
for v=v 
    for t1 = lag1
        for t2 = lag2
    [success, state] = runBicycleTestR(0,0,v,0, pi/6,0,0,p,K,0,t1,t2,1000,0);
    result(trial,1)= success;
    result(trial,2)= v;
    result(trial,3) = t1;
    result(trial,4) = t2;
    trial = trial+1;
        end
    end
    
end

%filter only for cases where the bike balanced. 
result = sortrows(result,1);
indx = find(result(:, 1));
successes = result(indx,:);
minSpeed = successes(1,2);
%we filter out successes that depend on 0 time lag. Comment this out to see
%all successes. 
indx = find( successes(:, 3));
successes = successes(indx,:);
indx = find( successes(:, 4));
successes = successes(indx,:);
%end of code intended to delete 0 time lag entries.  
end
