function plotBody(uG,kC)
    
    global body

    uBodyVertices = zeros(4,3);
    
    for i = 1:4
        [TB2G,TP2B,~,~,~,~,~,~,~,~] = generateTrMatrices(uG,[0 0 0 0],kC,i);
        TP2G = TB2G*TP2B;
        uBodyVertices(i,:)  = TP2G(1:3,4);
    end
    
    set(body,'Vertices',uBodyVertices,'Faces',[1 3 4 2]);
    
end

