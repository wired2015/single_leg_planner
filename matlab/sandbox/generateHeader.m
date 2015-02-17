function header = generateHeader(f)

    fid = fopen(f);
    s = fscanf(fid,'%s');
        
    len = length(s);
    
    index = 1;
    
    while s(index) ~= 'n' || index > len
        index = index + 1;
    end
    index = index + 1;
    disp(index)
    
    outputCount = 1;
    outputs = cell(1);
    output = '';
    
    index = index + 1;
    
    while index > len
        if s(index) == ',' || s(index) == ']'
            outputs{outputCount} = output;
            outputCount =  outputCount + 1;
            output = '';
            if s(index) == ']'
                break;
            end
        else
            output = [output s(index)];
        end
        index = index + 1;
    end
    
    while s(index) ~= '(' || index > len
        index = index + 1;
    end
    disp(index)
    
    argCount = 1;
    args = cell(1);
    arg = '';
    
    index = index + 1;
    
    while s(index) ~= ')' || index > len
        if s(index) == ','
            args{argCount} = arg;
            argCount =  argCount + 1;
            arg = '';
        else
            arg = [arg s(index)];
        end
        index = index + 1;
    end
    args{argCount} = arg;
    
    header = ['%' upper(f) '\n %Inputs:\n%\n'];
    
    for i = 1:argCount
        header = [header argDoc(args{i})];
    end
    
    header = [header '%\n%Outputs:\n'];
    
    for i = 1:outputCount
        header = [header argDoc(outputs{i})];
    end
    
end

function s = argDoc(arg)

    s = ['%' arg ':\n'];

end