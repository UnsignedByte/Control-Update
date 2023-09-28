function [] = plotController(x0,y0,v0,delta0,phi0, ...
                phi_dot0,psi0,p,  K, delta_offset, lag1,lag2,  numtime,  graph, cont)
%adapted from PlotMultipleControllersTogether to be usable with multiple
%timelags and to be usable as a function with many inputs.
%timelag specifies the lag between observing and acting on the state;
%version specifies that if 0, no time delays are desired, 1 specifies that
%only 
 
    [~, offset0] = runBicycleTestR(x0,y0,v0,delta0,phi0, ...
                phi_dot0,psi0, p,K, delta_offset,lag1,lag2, numtime,graph, cont);
 

    times = offset0(1:numtime,1);
    phi = offset0(1:numtime,4);
    delta = offset0(1:numtime,6);
    phidot = offset0(1:numtime,7);
    deltadot = diff(delta)./diff(times);
        
    
    subplot(2,2,1)
    hold on
    plot(times,phi);
    title('lean vs. time');
    xlabel('time (s)');
    ylabel('phi');
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
end 