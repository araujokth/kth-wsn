function matrix2C(name, matrix)

nRows = size(matrix, 1);
nColumns = size(matrix, 2);

for i=1:nRows 
    for j=1:nColumns
        disp(sprintf('%s[%d][%d]=%0.5f',name, i-1, j-1, matrix(i,j) ));
    end
end
end