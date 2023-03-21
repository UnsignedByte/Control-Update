%This runs all tests and displays tables for all user-determined
%parameters. Here, g is always 9.81, as it would be in practice, but can be
%tuned otherwise, and pl, pb, and ph are set to be the fairly reasonable
%proportions of a standard adult-sized bicycle.

%define parameters
   pg = linspace(9.8,9.81,1); %acceleration due to gravity
   pl = linspace(0.8, 1.2,3); %length of wheel base 
   pb = linspace(0.15,0.6, 3); %distance from rear wheel to COM projected onto ground
   ph = linspace(0.5, 1.1, 3); %height of COM in point mass model
% h is not the same as the height of COM of the bicycle, h is
% calculated to place the center of mass so that the point
% mass model and the real bicycle fall with the same falling frequency.
% see ABT Fall 2017 report for further discussion.
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
   
