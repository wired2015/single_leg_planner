%TR2ADJ Returns the adjunct matrix, A, based on the homogeneous
%transformation matrix, T. The adjunct matrix serves to transform the
%velocity from the one frame to another, as described by the homoegenous
%transformation matrix.
%
%Inputs:
%-T: The 4x4 homogeneous transformation matrix representing the
%transformation from one frame to another.
%Outputs:
%-A: The adjunct matrix that transforms velocity vectors from one frame to
%another.

function A = tr2Adj(T)
		
    R = T(1:3,1:3);
    A = [R skew(T(1:3,4))*R;
         zeros(3,3) R];
        
    function S = skew(v)
        S = [  0   -v(3)  v(2)
              v(3)  0    -v(1)
             -v(2) v(1)   0];