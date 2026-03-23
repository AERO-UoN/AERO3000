% Stability Derivatives
A = [-0.022   0.052   0.0012  -9.81;
     -0.64   -4.85   -0.61     0;
     -0.0048 -0.070  -1.79     0;
      0       0       1        0];

disp('A:');
disp(A);


% Eigenvalues
eigenvalues = eig(A);
disp('Eigenvalues:');
disp(eigenvalues);

slowIdx = [1,4];
fastIdx = [2,3];

% Short Period Approximation
Asp = A(fastIdx, fastIdx);
eigenvalues_sp = eig(Asp);
disp('Short Period Eigenvalues:');
disp(eigenvalues_sp);

% Phugoid Approximation

A11 = A(slowIdx, slowIdx);
A12 = A(slowIdx, fastIdx);
A21 = A(fastIdx, slowIdx);
A22 = A(fastIdx, fastIdx);

% Schur‐complement reduction
Aph = A11 - A12*(A22\A21);

eigenvalues_ph = eig(Aph);
disp('Phugoid Eigenvalues:');
disp(eigenvalues_ph);

% Stability Analysis
real_parts = real(eigenvalues);
if all(real_parts < 0)
    disp('The system is stable.');
else
    disp('The system is unstable.');
end