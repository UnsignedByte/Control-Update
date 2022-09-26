function[nav_score] = testSteerOffset(p, K, v0, delta_offset,x0, y0, delta0,phi0, phi_dot0, psi0,lag1, lag2, nonlinear, a,Km, visual)
%Written by Dylan Meehan (dem292), S18
%Test given controllers given a vector of commanded steer angles
numTimeSteps = length(delta_offset);
phi_offset = v0^2/p.l/p.g.*delta_offset; %steady state relation between phi & delta
%^this line is duplicated in runBicycleTest!!
if(nonlinear==1)
    [success,state, ~] = runBicycleTestNonlinear(x0,y0,v0,delta0,phi0, ...
       phi_dot0,psi0,p, K, Km,-1*delta_offset,a, lag1,lag2, numTimeSteps,  0);
   
else
    
 [success,state, ~] = runBicycleTestR(x0,y0,v0,delta0,phi0, ...
     phi_dot0,psi0,p, K,-1*delta_offset, lag1,lag2, numTimeSteps,  0);
end

         delta = state(:,6);
 if(visual)
 figure("Name", "SteerTestFor");
         subplot(2,2,1)
         hold on
         times = state(:,1);
         phi = state(:,4);
         phidot = state(:,7);
         deltadot = diff(delta)./diff(times);
         psi = state(:,5);
         x = state(:,2);
         y = state(:,3);

         subplot(2,2,1)
         plot(times,phi);
         hold on
         title('lean vs. time');
         xlabel('time (s)');
         ylabel('phi');
         subplot(2,2,2)
         ylim([-1 1.5])
         hold on
         subplot(2,2,2)
         hold on
         plot(times,phidot);
         title('lean rate vs. time');
         xlabel('time (s)');
         ylabel('phi-dot');
         subplot(2,2,3)
         hold on
         plot(times,delta);
         title('steer vs. time');
         xlabel('time (s)');
         ylabel('delta');
         subplot(2,2,4)
         hold on
         plot(times(1:end-1),deltadot);
         title('steer rate vs. time');
         xlabel('time (s)');
         ylabel('deltadot');

subplot(2,2,1)
        plot(times,phi_offset);
        hold on
         subplot(2,2,3)
        plot(times,delta_offset);
        hold on
        legend ("trajectory", "intended")
    end
    if (success)
    nav_score = norm(delta-delta_offset);
    else
        nav_score = 1000000;
    end
