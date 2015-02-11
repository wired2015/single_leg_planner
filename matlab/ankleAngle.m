function qA = ankleAngle(x,qAOld,kinematicConst)
    
    uDot = sherpaTTFKVel(x(7:9)',x(4:6)',kinematicConst);
    
    qA = atan2(uDot(2),uDot(1));
    
    if angDiff(qA+pi, qAOld) < angDiff(qA,qAOld)
        qAPos = qA+pi;
        if qAPos > pi
            qA = qA-pi;
        else
            qA = qAPos;
        end
    end
    
end

