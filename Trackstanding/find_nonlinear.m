function [KB, KM]= find_nonlinear(p, v0, delta_offset,x0, y0, delta0,phi0, phi_dot0, psi0,lag1, lag2, nonlinear, a, visual, num, seedings, param, factor)
Fin = zeros(seedings, 7);
disp(seedings);
parfor j  = 1:seedings
Res = zeros(1, 7); 
Res(1,1) = 10000;
tic
M = param;
for i= 1:num
    %K1= rand(1)*M(1,1)-M(2,1);
    K=[rand(1)*M(1,1)+M(2,1), rand(1)*M(1,2)+M(2,2),rand(1)*M(1,3)+M(2,3)];
    Km = [rand(1)*M(1,4)+M(2,4),rand(1)*M(1,5)+M(2,5),rand(1)*M(1,6)+M(2,6)];
    
   [nav]=testSteerOffset(p, K, v0, delta_offset,x0, y0, delta0,phi0, phi_dot0, psi0,lag1, lag2, nonlinear, a,Km, visual);
   if(nav < Res(1,1)) 
       %disp("hi!");
    Res(1,:) = [nav K Km];
    M(2,1:3) = K(1, 1:3);
    M(2,4:6) = Km(1,1:3);
    if(rand(1)* 0.7<0.5)
    M(1, :)  = M(1,:) *factor;
    else
        M(1, :)  = M(1,:) *1/factor;
    end
   end
end
toc
sortrows(Res,1); 
Fin(j,:) = Res(1,:);
end
sortrows(Fin,1); 
KB = Fin(1,2:4);
KM = Fin(1,5:7);
Fin
%sortrows(Fin,1)
end