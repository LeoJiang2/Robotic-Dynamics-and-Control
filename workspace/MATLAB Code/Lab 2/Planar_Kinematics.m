


syms t2 t3 t2dot t3dot lc2 l2 lc3 m2 m3 I2zz I3zz

%Current joint states
q = [t2;t3];
q_dot = [t2dot; t3dot];

%Forward kinematics, end-effector position
x2 = lc2*cos(t2)
y2 = lc2*sin(t2)
x3 = l2*cos(t2)+lc3*cos(t2+t3)
y3 = l2*sin(t2)+lc3*sin(t2+t3)

% %End-effector Jacobian (linear velocity)
% Jv = [[-L1*sin(q(1))-L2*sin(q(1) + q(2)) -L2*sin(q(1) + q(2))];
%       [ L1*cos(q(1))+L2*cos(q(1) + q(2))  L2*cos(q(1) + q(2))];];

%%Jv2 = [[diff(x2, q(1)) diff(y2,q(1))];
      %[diff(x2, q(2)) diff(y2,q(2))];]

%Jv3 = [[diff(x3, q(1)) diff(y3,q(1))];
     % [diff(x3, q(2)) diff(y3,q(2))];]
Jv2 = simplify(jacobian([x2; y2], q))
Jv3 = simplify(jacobian([x3; y3], q))

%I2 = [[0 0 0];
  %    [0 0 0];
   %   [0 0 I2zz]];

%I3 = [[0 0 0];
    %  [0 0 0];
     % [0 0 I3zz]];
I3 = I3zz
I2 = I2zz
% R01 = [[1 0 0];
%       [0 1 0];
%       [0 0 1]];
% R02 = [[1 0 0];
%       [0 1 0];
%       [0 0 1]];

R01 = [[cos(t2) -sin(t2)];
      [sin(t2) cos(t2)];];
R02 = [[cos(t2+t3) -sin(t2+t3)];
      [sin(t2+t3) cos(t2+t3)];];

Jw2 = 1;
Jw3 = 1;

Kv = simplify(1/2*m2*q_dot.'*Jv2.'*Jv2*q_dot + 1/2*m3*q_dot.'*Jv3.'*Jv3*q_dot)
Kw = simplify(1/2*q_dot.'*Jw2.'*R01*I2*R01.'*Jw2*q_dot + 1/2*q_dot.'*Jw3.'*R02*I3*R02.'*Jw3*q_dot)

