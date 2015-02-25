%LININTERP Returns a Cartesian path that is linearly interpolated between
%two points.
%
%Inputs:
%s1: The initial state that is represented by a 1x8 vector. The vector has
%the form [t x y z xDot yDot zDot interpBoolean]
%s2: The goal state that is represented by a 1x8 vector. The vector has
%the form [t x y z xDot yDot zDot interpBoolean]
%N: The number of waypoints to be placed in the path.
%
%Outputs:
%path: A 8xN matrix that returns a path from s2 to s1.
%
%linInterp.m
%author: wreid
%date: 20150224

function path = linInterp(s1,s2,N)
    
    tStart = s1(1);
    cost2Go = s1(2);
    n1 = s1(3:5);
    n2 = s2(3:5);

    oldState = n1;
    path = zeros(N,9);
    for i = 1:N
        newState = (n1+i/N*(n2-n1));
        cost2Go = cost2Go+norm(newState-oldState);
        path(i,:) = [tStart+i/N cost2Go newState 0 0 0 true]; 
        oldState = newState;
    end
    
end

