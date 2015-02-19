function s = plotGround(workspace,zGround)

    xmin = workspace(1);
    xmax = workspace(2);
    ymin = workspace(3);
    ymax = workspace(4);
    
    tile1color = [0.5 1 0.5];  % light green
    tile2color = [1 1 1];  % white
    tilesize = 0.2;
    
    % create a colored tiled floor
    xt = xmin:tilesize:xmax;
    yt = ymin:tilesize:ymax;
    Z = zGround*ones( numel(yt), numel(xt));
    C = zeros(size(Z));
    [r,c] = ind2sub(size(C), 1:numel(C));
    C = bitand(r+c,1);
    C = reshape(C, size(Z));
    C = cat(3, tile1color(1)*C+tile2color(1)*(1-C), ...
        tile1color(2)*C+tile2color(2)*(1-C), ...
        tile1color(3)*C+tile2color(3)*(1-C));
    [X,Y] = meshgrid(xt, yt);
    s = surface(X, Y, Z, C, ...
        'FaceColor','texturemap',...
        'EdgeColor','none',...
        'CDataMapping','direct');
end