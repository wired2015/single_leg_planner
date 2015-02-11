function delA = ankleDiff(x1,x2,qAOld,kinematicConst)
%ankleDiff Calculates the required ankle motion between two states.
%   The amount by which an ankle joint has to move between two states is
%   determined by the hip and upper thigh velocities at each state.

    qA1 = ankleAngle(x1,qAOld,kinematicConst);
    qA2 = ankleAngle(x2,qA1,kinematicConst);
    delA = angDiff(qA1,qA2);

end

