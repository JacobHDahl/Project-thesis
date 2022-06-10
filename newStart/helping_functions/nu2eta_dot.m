function eta_dot = nu2eta_dot(nu,theta)
u = nu(1);
w = nu(2);
q = nu(3);

pn_dot = cos(theta)*u + sin(theta)*w;
h_dot = sin(theta)*u - cos(theta)*w;
theta_dot = q;

eta_dot = [pn_dot;h_dot;theta_dot];

end

