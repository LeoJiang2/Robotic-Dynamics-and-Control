% Units are in radians and meters
al1 = -sym(pi)/2;
al2 = 0;
al3 = 0;
a1 = 0;
a2 = 0.254;
a3 = 0.254;
d1 = 0.254;
d2 = 0;
d3 = 0;

c1 = -1;
c2 = 1;
c3 = sym(pi)/2;
c4 = -sym(pi)/2;
syms mt1 mt2 mt3
t1 = mt1;
t2 = mt2 + c4;
t3 = c1*mt2 + c2*mt3 + c3;

% DH1 transformation
A1 = [[cos(t1) -sin(t1)*cos(al1) sin(t1)*sin(al1) a1*cos(t1)];
      [sin(t1) cos(t1)*cos(al1) -cos(t1)*sin(al1) a1*sin(t1)];
      [0 sin(al1) cos(al1) d1];
      [0 0 0 1]];

A2 = [[cos(t2) -sin(t2)*cos(al2) sin(t2)*sin(al2) a2*cos(t2)];
      [sin(t2) cos(t2)*cos(al2) -cos(t2)*sin(al2) a2*sin(t2)];
      [0 sin(al2) cos(al2) d2];
      [0 0 0 1]];

A3 = [[cos(t3) -sin(t3)*cos(al3) sin(t3)*sin(al3) a3*cos(t3)];
      [sin(t3) cos(t3)*cos(al3) -cos(t3)*sin(al3) a3*sin(t3)];
      [0 sin(al3) cos(al3) d3];
      [0 0 0 1]];

H03 = simplify(A1*A2*A3)