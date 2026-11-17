function T = Path_matrix_Patu(param)

if nargin > 0
    T = kron(param.Tz, eye(3));     % general: built from the path matrix of the model
    return
end

T = [1     0     0     0     0     0     0     0     0     0     0     0
     0     1     0     0     0     0     0     0     0     0     0     0
     0     0     1     0     0     0     0     0     0     0     0     0
     1     0     0     1     0     0     0     0     0     0     0     0
     0     1     0     0     1     0     0     0     0     0     0     0
     0     0     1     0     0     1     0     0     0     0     0     0
     1     0     0     1     0     0     1     0     0     0     0     0
     0     1     0     0     1     0     0     1     0     0     0     0
     0     0     1     0     0     1     0     0     1     0     0     0
     1     0     0     0     0     0     0     0     0     1     0     0
     0     1     0     0     0     0     0     0     0     0     1     0
     0     0     1     0     0     0     0     0     0     0     0     1];