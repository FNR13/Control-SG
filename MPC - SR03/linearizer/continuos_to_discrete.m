clc
clear

load linear_model.mat

Ts = 0.020;
discrete_model = c2d(lin_model,Ts);

Ad = discrete_model.A;
Bd = discrete_model.B(:,1:3);
Cd = discrete_model.C;
Bdd = discrete_model.B(:,4:5);

fprintf('Ad << ')
printFormattedMatrix(Ad)
fprintf('\n')

fprintf('Bd << ')
printFormattedMatrix(Bd)
fprintf('\n')

fprintf('Cd << ')
printFormattedMatrix(Cd)
fprintf('\n')

fprintf('Bdd << ')
printFormattedMatrix(Bdd)

function printFormattedMatrix(matrix)
    [n, m] = size(matrix);
    
    for i = 1:n
        for j = 1:m
            if i == n && j == m
                fprintf('%f', matrix(i, j));
            else
                fprintf('%f, ', matrix(i, j));
            end
        end
        if i ~= n 
            fprintf('\n    ');
        end
    end
    fprintf(';\n');
end
