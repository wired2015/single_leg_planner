function panHeight = getPanHeight( bodyHeight,kinematicConst )
%GETPANHEIGHT Summary of this function goes here
%   Detailed explanation goes here

    [~,~,~,~,~,~,~,~,~,~,~,B2PZOffset,~,~,~,~] = extractKinematicConstants(kinematicConst);
    panHeight = -(bodyHeight + B2PZOffset);

end

