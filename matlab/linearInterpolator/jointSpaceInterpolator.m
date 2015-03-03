function jointSpaceInterpolator(s1B,s2B,kC,jointLimits,TP2B,dt)

    alphaDotMax = jointLimits(2,6);
    betaDotMax = jointLimits(2,7);
    gammaDotMax = jointLimits(2,8);
    alphaDotMin = jointLimits(2,6);
    betaDotMin = jointLimits(2,7);
    gammaDotMin = jointLimits(2,8);
    
    u1B = s1B(1:3);
    u2B = s2B(1:3);

    TB2P = trInv(TP2B);
    u1P = tr(u1B',TB2P);
    u2P = tr(u2B',TB2P);
      
    hold on
    axis equal
    plot3(u1P(1),u1P(2),u1P(3),'g*');
    plot3(u2P(1),u2P(2),u2P(3),'r*');
    
    q2 = sherpaTTIK(u2P,kC,jointLimits);
    path = [];
    betaDotVec = [];
    gammaDotVec = [];
    t = [];
    uCurr = u1P;
    %dsMax = ;
    Ds = 100;
    tCurr = 0;
    while Ds > 0.3*dt
        qCurr = sherpaTTIK(uCurr,kC,jointLimits);
        [Ds,eta] = heuristicCylindrical(qCurr,q2,kC,jointLimits);
        r = legRadius(qCurr(2),qCurr(3),kC);
        
        if q2(2)-qCurr(2) > 0
            betaDot = betaDotMax;
        else
            betaDot = betaDotMin;
        end
        
        if q2(3)-qCurr(3) > 0
            gammaDot = gammaDotMax;
        else
            gammaDot = gammaDotMin;
        end
        
        if q2(1)-qCurr(1) > 0
            alphaDot = alphaDotMax;
        else
            alphaDot = alphaDotMin;
        end
       
        gammaDotConstrained = getConstrainedGammaDot(kC,betaDot,qCurr);
        betaDotConstrained = getConstrainedBetaDot(kC,gammaDot,qCurr);
        
        if gammaDotConstrained > gammaDotMax
            gammaDot = gammaDotMax;
            betaDot = getConstrainedBetaDot(kC,gammaDot,qCurr);
        elseif gammaDotConstrained < gammaDotMin
            gammaDot = gammaDotMin;
            betaDot = getConstrainedBetaDot(kC,gammaDot,qCurr);
        elseif betaDotConstrained > betaDotMax
            betaDot = betaDotMax;
            gammaDot = getConstrainedGammaDot(kC,betaDot,qCurr);
        elseif betDotConstrained < betaDotMin
            betaDot = betaDotMin;
            gammaDot = getConstrainedGammaDot(kC,betaDot,qCurr);
        else
            gammaDot = gammaDotConstrained;
        end
        
        rDot = -kC.l3*betaDot*sin(qCurr(2))-gammaDot*kC.l5*sin(kC.zeta+qCurr(3));
        thetaDot = alphaDot;
        ds = sqrt(rDot^2+r^2*thetaDot^2);
        uCurr(1) = uCurr(1) + dt*ds*cos(eta);
        uCurr(2) = uCurr(2) + dt*ds*sin(eta);
        %plot3(uCurr(1),uCurr(2),uCurr(3),'b.')
        path = [path; uCurr'];
        gammaDotVec = [gammaDotVec gammaDot];
        betaDotVec = [betaDotVec betaDot];
        tCurr = tCurr+dt;
        t = [tCurr t];
    end
    figure(1)
    plot3(path(:,1),path(:,2),path(:,3),'b.');
    figure(2)
    plot(t,gammaDotVec);
    figure(3)
    plot(t,betaDotVec)
        
end