function [E,A_P,B_P] = ImplicitLinmod(MY_FUN, XDOTo, Xo, Uo, DXDOT, DX, DU)

%Obtain number of states and controls
n = length (XDOTo);
m = length (Uo);

%-------------------------CALCULATE E MATRIX--------------------
%Initialize the E matrix
E = zeros(n,n);
%fill in each element of the matrix individually
for i=1:n
    for j=1:n
        %Obtain the magnitude of the pertubation to use.
        dxdot = DXDOT(i,j);
        %Define pertubation vector. The current column determines which element of xdot we are pertubing.
        xdot_plus = XDOTo;
        xdot_minus = XDOTo;

        xdot_plus(j) =xdot_plus(j) + dxdot;
        xdot_minus(j) = xdot_minus(j) - dxdot;

        %Calculate F(row) (xdot_plus,xo, uo)
        F = feval(MY_FUN, xdot_plus, Xo, Uo);
        F_plus_keep = F(i);
        %Calculate F(row) (xdot minus,xo, uo)
        F = feval (MY_FUN, xdot_minus, Xo, Uo);
        F_minus_keep = F(i);

        % Calculate E(row,col)
        E(i,j) = (F_plus_keep - F_minus_keep)/(2*dxdot);
    end
end

%----------------------CALCULATE A_P MATRIX---------------------------
%Initialize the A_P matrix
A_P= zeros(n,n);

%fill in each element of the matrix individually
for i=1:n
    for j=1:n
        %Obtain the magnitude of the pertubation to use.
        dx = DX(i,j);
        %Define pertubation vector. The current column determines which
        %element of x we are pertubing.
        x_plus = Xo;
        x_minus = Xo;
       
        x_plus(j) = x_plus(j) + dx;
        x_minus (j) = x_minus(j) - dx;

        %Calculate F(row) (xdoto,x_plus,uo)
        F = feval(MY_FUN, XDOTo, x_plus, Uo);
        F_plus_keep = F(i);
        %Calculate F(row) (xdoto,x_minus,uo)
        F = feval(MY_FUN, XDOTo, x_minus, Uo);
        F_minus_keep = F(i);
       
        %Calculate Aprime(row,col)
        A_P(i,j) = (F_plus_keep - F_minus_keep)/(2*dx);
    end
end

%----------------------CALCULATE B_P MATRIX-----------------------------
%Initialize the B_P matrix
B_P = zeros(n,m);

%fill in each element of the matrix individually
for i=1:n
    for j=1:m
        %Obtain the magnitude of the pertubation to use.
        du = DU(i,j);
       
        %Define pertubation vector. The current column determines which %element of u we are pertubing.
        u_plus = Uo;
        u_minus = Uo;
       
        u_plus(j) = u_plus(j) + du;
        u_minus(j) = u_minus(j) - du;

        %Calculate F(row) (xdoto, xo, u_plus)
        F = feval (MY_FUN, XDOTo, Xo,u_plus);
        F_plus_keep = F(i);
        
        %Calculate F(row) (xdoto, xo, u_minus)
        F = feval (MY_FUN, XDOTo, Xo, u_minus);
        F_minus_keep = F(i);
       
        %Calculate Bprime (row,col)
        B_P(i,j) = (F_plus_keep - F_minus_keep)/(2*du);
    end
end
