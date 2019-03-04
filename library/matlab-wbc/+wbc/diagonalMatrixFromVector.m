function D = diagonalMatrixFromVector(d)

    % DIAGONALMATRIXFROMVECTOR coverts a vector into a diagonal matrix.
    %
    % FORMAT: D = diagonalMatrixFromVector(d)
    %
    % INPUT:  - d = [n * 1] vector;
    %
    % OUTPUT: - D = [n * n] diagonal matrix.
    %
    % Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
    %          
    %          all authors are with the Italian Istitute of Technology (IIT)
    %          email: name.surname@iit.it
    %
    % Genoa, Dec 2017
    %

    %% --- Initialization ---
    
    D = diag(d);
end