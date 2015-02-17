%MOTIONDUETOBODYRATE Calculates the wheel rate, omega, and the steering
%joint orientation, phi, given a platform Cartesian velocity and angular
%velocity relative to the inertial frame.
%
%Inputs:
%-vB: A 1x3 vector describing the Cartesian [xDot yDot zDot] velocities
%relative to the inertial frame.
%-omegaB: A 1x3 vector descring the angular rates of the platform relative
%to the inertial frame.
%
%Outputs:
%-omega: The required wheel rate to meet the body rate.
%-phi: The required steering joint orientation to allow for the body rate
%input.

function [omega,phi] = motionDueToBodyRate(vB,omegaB,kC)

    %Find the instantaneous centre of rotation relative to the body
    %coordinate frame.
    sB = calcICR(vB,omegaB);
    
    %Find the transformation matrix from the body frame to the
    %wheel steering frame. 
    [~,TP2B,TI2P,TJ2I,TO2J,TQ2O,TR2Q,TS2R,TW2S,~] = generateTrMatrices(uG,q,kC,legNum);
    TR2B = TP2B*TI2P*TJ2I*TO2J*TQ2O*TR2Q*TS2R;
    TB2R = trInv(TR2B);
    
    %Transform the ICR from the body frame to the wheel steering frame.
    [sR,~] = tr(sB,TB2R);

    %Calculate the desired ankle angle.
    phi = atan2(sR(2),sR(1));

    %Make sure that the ankle angle stays within -pi/2 and pi/2.
    if phi > pi/2
        phi = phi - pi;
    elseif phi < -pi/2
        phi = phi + pi;
    elseif isnan(phi)
        phi = 0;
    end
    
    %Find the transformation matrix from the body frame to the wheel frame.
    TW2B = TR2B*TW2R;
    TB2W = trInv(TW2B);

    %Find the ICR relative to the wheel coordinate frame.
    sW = tr(sB,TB2W);

    %Determine the sign of the velocity by looking at the sign of the
    %distance vector that connects the centre of each wheel and the
    %ICR.
    if sW(3) >= 0
        sign = 1;
    else
        sign = -1;
    end
    
    if omegaB ~= [0 0 0]
        omega = sign*omegaB*norm(sW);
    else
        omega = sign*norm(v);
    end

end