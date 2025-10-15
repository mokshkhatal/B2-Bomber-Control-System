clear
clc
close all
%Straight and level condition
Xdoto = [
        0
        0
        0
        0
        0
        0
        0
        0
        0
        0
        0
        0
        ];

Xo =    [
        84.061206
        -0.040431
        4.042791
        0.000033
        -0.034194
        0.000090
        -0.000003
        0.049636
        0.034465
        0
        0
        0
        ];

Uo =    [
        0.001119
        -0.004363
        -0.001000
        0.063266
        0.062101
        ];

%Define the pertubation matrices (ie how much we perturb each function in
%each direction)
dxdot_matrix = 10e-6*ones (12,12);
dx_matrix = 10e-6*ones (12,12);
du_matrix = 10e-6*ones (12,5);

[E, Ap, Bp] = ImplicitLinmod(@B2_6DOF_model_implicit, Xdoto, Xo, Uo, dxdot_matrix, dx_matrix, du_matrix);

%Calculate the A and B matrices
A = -inv(E)*Ap;
B = -inv(E)*Bp;

save('coefficients_of_linearization.mat','A','B');

disp(A);
disp(B);
