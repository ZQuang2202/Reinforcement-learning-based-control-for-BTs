function a = gradPhi3(x)
    a = [0              , 0;...
         0              , 0;...
         0              , 0;...
         0              , 0;...
         0              , 0
         2*x(5)*x(1)^2  , 0;...
         0              , 2*x(6)*x(2)^2;...
         0              , 0;...
         0              , 0;...
         0              , 0;...
         x(2)^2*x(4)    , 0;...
         x(3)^2*x(6)    , x(3)^2*x(5);...
         0              , 0;...
         0              , 0;...
         x(4)*x(6)*x(7)*x(8), x(4)*x(5)*x(7)*x(8);...
         0              , 0;...
         0              , 0;...
         4*x(5)^3       , 0];
% a = [    0              , 0;...
%          0              , 0;...
%          0              , 0;...
%          0              , 0;...
%          0              , 0
%          2*x(5)*x(1)^2  , 0;...
%          0              , 2*x(6)*x(2)^2;...
%          0              , 0;...
%          0              , 0;...
%          0              , 0;...
%          x(2)^2*x(4)    , 0;...
%          x(3)^2*x(6)    , x(3)^2*x(5);...
%          0              , 0;...
%          0              , 0;...
%          0              , 0;...
%          0              , 0;...
%          0              , 0;...
%          4*x(5)^3       , 0];
end