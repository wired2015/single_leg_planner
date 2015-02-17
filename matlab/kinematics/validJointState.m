%VALIDJOINTSTATE Checks if a leg's joint state complies with the leg's angular
%position and rate limits.
%
%Inputs:
%-state:       A 1x6 vector containing the joint state to be checked.
%-jointLimits: A 2x6 vector containing the angular position and rate
%              limits.
%Outputs:
%-valid:       A logical value indicating whether or not state is valid.
%
%validJointState.m
%author: wreid
%date:   20151402

function valid = validJointState(state,jointLimits)
    
    if state(1) < jointLimits(1,1) || state(1) > jointLimits(2,1) ||...
       state(2) < jointLimits(1,2) || state(2) > jointLimits(2,2) ||...
       state(3) < jointLimits(1,3) || state(3) > jointLimits(2,3) ||...
       state(4) < jointLimits(1,4) || state(4) > jointLimits(2,4) ||...
       state(5) < jointLimits(1,5) || state(5) > jointLimits(2,5) ||...
       state(6) < jointLimits(1,6) || state(6) > jointLimits(2,6)
    
        valid = false;
    else
        valid = true;
    end
end

