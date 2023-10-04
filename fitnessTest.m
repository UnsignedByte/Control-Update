%The point of this is to automate and find preferable controllers for any
%set of parameters for the bike, p, for any initial velocity, for any set
%of timelags. As such, it is a set of tests that parameterizes on p, uses a
%master controller [11.1, 2.4, -6.6] (still figuring out why that is a
%master controller, but hopefully many tests run on many different
%parameters will explain why).

%Testing protocol: Input p, v0. Find: 1) best lqr controller, 2) 2 other
%random successful lqr controllers, 3) grid search controller, 4) old lqr controller. Then, test for 1)
%speed of stability, 2) max amplitude of delta_dot, 3) largest stabilizing
%actuation flaw against controllers 1,2,3 and [11, 2,-6]

function [specs] = fitnessTest(p, v0, details)
% p: parameters
% v0: initial velocity
% details: whether to show detailed graphs
delta0 = 0;
phi0 = pi/6;
phi_dot0 = 0;
x0 = 0;
y0 = 10;
psi0 = 0;
v0 = v0;

% Save to custom file in results folder
warning('off', 'MATLAB:MKDIR:DirectoryExists');
fold = fullfile(pwd, "results", datestr(datetime, 'yyyy_mm_dd'), "fitness", "l"+p.l+ "_b"+ p.b+"_h" + p.h+"_g" + p.g);
mkdir (fold)
try
    rmdir (fold, 's')
    mkdir (fold)
catch ex
end

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
    fig = figure("Name", "Parameters" +p.b+"," + p.h+ ","+ p.l, 'visible', 'off');
    hold off
    subplot(2,2,1)
    for a = 1:5
        [success,states, stable] = runBicycleTestR(x0,y0,v0,delta0,phi0, phi_dot0,psi0,p,  specs(a,1:3), 0, t1,t2, 10000,0,0);
        specs(a, 4) = stable;

        plotController(x0,y0,v0,delta0,phi0, ...
            phi_dot0,psi0,p,  specs(a,1:3), 0, t1,t2,  500,  0, 1);
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
    k_3 = specs(:,3);
    stabilize = specs(:,4);
    speed = specs(:,5);
    tl1 = specs(:,6);
    tl2 = specs(:,7);
    tls = specs(:,8);
    labels = ["old_lqr","best_lqr","rand_lqr_1","rand_lqr_2","grid_tuned"];
    T = table(k_1,k_2,k_3,stabilize, speed, tl1,tl2,tls, 'RowNames', labels);
    legend(labels);

    hold off

    writetable(T, fullfile(fold, "data.csv"));

    saveas(fig, fullfile(fold, "summary.svg"));
    saveas(fig, fullfile(fold, "summary.png"));

    if details
        dfold = fullfile(fold, "detailed");
        mkdir (dfold)
        for a = 1:5
            df = fullfile(dfold, labels(a));
            mkdir (df)
            delays = fopen(fullfile(df, "delays.txt"), 'w');
            fprintf(delays, specs(a, 6)+"\n"+specs(a, 7));
            fclose(delays)
            % simulate the test again with the proper delays
            [~, states, ~, motCommands, phi_offset] = runBicycleTestR(x0,y0,v0,delta0,phi0, phi_dot0,psi0,p, specs(a, 1:3), 0, specs(a, 6), specs(a, 7), 1000,0,1);
            %% Stolen from animateBike.m and repurposed to save files
        
            %unpack state variables over time:
            tarray = states(:,1);
            x=states(:,2);
            y=states(:,3);
            z=zeros(size(x0));
            phi=states(:,4);
            psi=states(:,5);
            delta=states(:,6);
            phi_dot=states(:,7);
            v = states(:,8);
            
            %%plot bicycle states
            fig = figure('Name', "Steer vs. Time", 'visible', 'off');
            hold on
            subplot(2,2,1)
            plot(tarray,phi, tarray,phi_offset);
            title('lean vs. time');
            xlabel('time (s)');
            ylabel('phi');
            legend("lean","desired lean");
            subplot(2,2,2)
            plot(tarray,phi_dot);
            title('lean rate vs. time');
            xlabel('time (s)');
            ylabel('phi-dot');
            subplot(2,2,3)
            plot(tarray,delta, tarray, 0);
            title('steer vs. time');
            xlabel('time (s)');
            ylabel('delta');
            legend("steer", "desired steer")
            subplot(2,2,4)
            plot(tarray,motCommands);
            title('acceleration commands vs. time');
            xlabel('time (s)');
            ylabel('delta-dot');
            hold off;

            saveas(fig, fullfile(df, "steer_v_time.svg"));
            saveas(fig, fullfile(df, "steer_v_time.png"));
            
            fig = figure('Name', "bicycle trajectory", 'visible', 'off');
            hold on;
            subplot(2,1,1)
            plot(x,y);
            axis equal
            title('bicycle trajectory');
            xlabel('x position');
            ylabel('y position');
            subplot(2,1,2)
            psid = diff(psi)/diff(tarray);
            plot(tarray(1:end-1),psid);
            title('yaw dot vs. time');
            xlabel('time (s)');
            ylabel('yaw dot');
            hold off;

            saveas(fig, fullfile(df, "yawdot_v_time.svg"));
            saveas(fig, fullfile(df, "yawdot_v_time.png"));
        end
    end
catch ex
    %global try/catch failure, printed with error and stack information
    fprintf("\nfailed with error:\n%s", getReport(ex))
end