syms x y z

c1 = -1;
c2 = 1;
c3 = sym(pi)/2;
c4 = -sym(pi)/2;

thetaa = atan2((z-0.254),(x^2+y^2)^0.5);
r = (x^2+y^2+(z-0.254)^2)^0.5;
c = (0.254^2-(r/2)^2)^0.5;
thetab = atan2(2*c,r);

theta1 = atan2(y,x);
theta2 = -(thetaa+thetab);
theta3 = 2*thetab;

motortheta1 = theta1
motortheta2 = simplify(vpa(theta2 + pi/2))
motortheta3 = simplify(vpa(theta3 + motortheta2 - pi/2))
