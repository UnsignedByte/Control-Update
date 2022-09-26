function [success, allStates,stability] = runBicycleTestR(x0,y0,v0,delta0,phi0, ...
                phi_dot0,psi0,p, K, delta_offset, lag1,lag2, numTimeSteps,  graph)
%see 2017 team function runBicycleTest for standard documentation; this is
%mostly a copy-and paste reapplication of the code until it works well
%enough to refactor the original code. 


stabled = -1; 
%simulation variables
timestep = 0.01;  %seconds

threshold2 = lag2./timestep; %observability delay
threshold = lag1 ./timestep +threshold2;%determines how long we wait between timesteps to change control signal.
p.pause = timestep;
%handle scalar(constant) or vector (variable) delta_offset
if isscalar(delta_offset)
    delta_offset = ones(numTimeSteps,1).*delta_offset; 
end
%delta_offset
%calculate lean correction for desired steer
phi_offset = v0.^2/p.l/p.g.*delta_offset; %steady state relation between phi & delta
    %assumes constant velocity (v0)

%initialize arrays of state and actuator data
count = 1;
success = 1;
tstart = 0;
currentState = [tstart x0 y0 phi0 psi0 delta0 phi_dot0 v0];

%intialize arrays before going into loop
allStates = zeros(numTimeSteps,8);
motCommands = zeros(numTimeSteps,1);

%add initial values to arrays
allStates(1,:) = currentState; %keep track of the state at each timestep
motCommands(1) = 0; %command steer angle rate (delta dot) of front motor
storedState = currentState;
change = 0;
u_init =0;
 while (count < numTimeSteps)    
    count = count+1;
   %if the time lag between the last observed change and the time to
   %actuate has elapsed, calculate the control change based on the last
   %observed state
   if(change<=(threshold2 +1))
       storedState = currentState;
   end
   if(change>=threshold)
   [zdot1, u1] =rhs_trackstand(storedState,p,K, delta_offset(count), 0); 
   %storedState=currentState;
   u_init = u1;
   %display(u);
   change=0;
   end
   %add the current control change signal to the motor commands.
   [zdot, u] = rhs_trackstand_init(currentState,p,[0, 0, 0], u_init, delta_offset(count), 0,true); 
  % display(u);
 % fprintf("time  :  " + count+"  ");
   
    %calculate derivatives based on EOM
    %also calculate u=delta_dot, the desired steer rate the keep balance
    %(count+1) gives rhs the delta_offset that the bike should be at the 
    %next timestep
 
   % If the lean angle is too high, the test should count as a failure,
   % ie, the bicycle falls over
   phi = currentState(4);
   if abs(phi)>=pi/4
       %fprintf('Bike has Fallen; Test Failure\n')
       success = 0; %failure
       break;
   end
   
   %update state

   previousState = currentState;
   currentState(1,1) = previousState(1,1) + timestep; %update time
   currentState(1,2:end) = previousState(1,2:end) + zdot*timestep; %Euler integrate
   allStates(count,:) =  currentState; %record state
   motCommands(count) = u; %record motor commands
  if(count>3)
   [stability]= stable(currentState, delta_offset(count), phi_offset(count), previousState);
 if(stability)
       stabled = count; 
      % break;
 end
  end
  change = change+1; 
 end
 if stabled < 0
    % success = 0;
    % fprintf("\n NOT STABLE");
 end 
 stability =stabled;
 %fprintf("\n count= " + K(1)+","+K(2)+" ," +K(3)+ "  "+ stabled+"\n");
 if graph == 1
     clf
     animateBike(allStates,p,motCommands,delta_offset, phi_offset);
     %animateBike is a rename of simulateBike from older MATLAB versions
 end

end