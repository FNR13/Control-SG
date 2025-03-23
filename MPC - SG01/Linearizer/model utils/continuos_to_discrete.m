clc
clear

load linear_model.mat

Ts = 0.020;
discrete_model = c2d(lin_model,Ts);

fprintf('// Define dynamics matrix \n')
fprintf('Ad << ')
printFormattedMatrix(discrete_model.A)
fprintf('\n')

fprintf('// Define inputs matrix \n')
fprintf('Bd << ')
printFormattedMatrix(discrete_model.B(:,1:2))
fprintf('\n')

fprintf('// Define Output matrix \n')
fprintf('Cd << ')
printFormattedMatrix(discrete_model.C)
fprintf('\n')

fprintf('// Define Disturbances matrix \n')
fprintf('Bdd << ')
printFormattedMatrix(discrete_model.B(:,3:4))

function printFormattedMatrix(matrix)
    [n, m] = size(matrix);

    for i = 1:n
        for j = 1:m
            if i == n && j == m
                fprintf('%.4f', matrix(i, j));
            else
                fprintf('%.4f, ', matrix(i, j));
            end
        end
        if i ~= n 
            fprintf('\n    ');
        end
    end
    fprintf(';\n');
end
