function node = makeNode(id,parent,cost,q,qDot,u,uDot)
    node = struct('id',id,'parent',parent,'cost',cost,'q',q,'qDot',qDot,'u',u,'uDot',uDot);
end