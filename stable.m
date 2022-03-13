function [isStable]=stable(currentState,currentMot, previousState,previousMot)
%Tests whether motion derivatives approach zero, so that we don't need to
%manually check whether the bicycle is actually in a stable state or is
%oscillating. Small decimals used to avoid float rounding problems. 
isStable = true;    
if(currentState(1,4)> 0.001||currentState(1,4)<-0.001)
        isStable = false;
end 
if(currentState(1,6)> 0.001||currentState(1,6)<-0.001) 
    isStable = false; 
end 
if(currentState(1,7)> 0.001||currentState(1,7)<-0.001) 
    isStable = false; 
end 
if(previousState(1,4)> 0.001||previousState(1,4)<-0.001)
        isStable = false;
end 
if(previousState(1,6)> 0.001||previousState(1,6)<-0.001)
    isStable = false; 
end 
if(previousState(1,7)> 0.001||previousState(1,7)<-0.001)
    isStable = false; 
end 
if(currentMot>0.001 || previousMot >0.001||currentMot<-0.001 || previousMot <-0.001) 
    isStable = false;
end
end 
    