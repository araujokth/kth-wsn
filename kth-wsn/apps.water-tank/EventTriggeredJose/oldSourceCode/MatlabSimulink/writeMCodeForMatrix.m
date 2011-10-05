function writeMCodeForMatrix(matrix)

fprintf(' --------- %% M-Code %% --------- \n\n');


fprintf('matrix = [');
for i=1:size(matrix,1)
    for j=1:size(matrix,2)
        fprintf('%10.50f ', matrix(i,j));
    end
    if i<size(matrix,1)
        fprintf(';\n');
    end
end
fprintf('];\n\n');