%This runs all tests and displays tables for all user-determined
%parameters. Here, g is always 9.81, as it would be in practice, but can be
%tuned otherwise, and pl, pb, and ph are set to be the fairly reasonable
%proportions of a standard adult-sized bicycle.
   pg = linspace(9.8,9.81,1);
   pl = linspace(0.8, 1.2,3);
   pb = linspace(0.15,0.6, 3); 
   ph = linspace(0.5, 1.1, 3); 
   v0 = 3;
   for g = pg 
       for l = pl 
           for b = pb
               for h = ph 
                   p.h = h;
                   p.b = b;
                   p.l = l; 
                   p.g = g;
                   x = fitnessTest(p, v0);
               end 
           end
       end
   end
   
