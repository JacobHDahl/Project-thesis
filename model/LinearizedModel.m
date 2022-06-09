function x_dot_lin = LinearizedModel(x_lin, u_lin, ConstStruct)
%LINEARIZEDMODEL is a linearized model of the NonLinear model
% linearized around trim

g = 9.81;
u_trim = ConstStruct.u_trim;
w_trim = ConstStruct.w_trim;
theta_trim = ConstStruct.theta_trim;


[Xu, Xw, Xq, XdeltaE, XdeltaT, Zu, Zw, Zq, ZdeltaE, Mu, Mw, Mq, MdeltaE] = LinearParamCalculation(ConstStruct);

A_lin = [Xu, Xw, Xq, -g*cos(theta_trim), 0;
        Zu, Zw, Zq, -g*sin(theta_trim), 0;
        Mu, Mw, Mq,      0,             0;
        0,  0,  1,       0              0;
        sin(theta_trim), -cos(theta_trim), 0, u_trim*cos(theta_trim) + w_trim*sin(theta_trim), 0];
B_lin = [XdeltaE, XdeltaT;
        ZdeltaE,    0    ;
        MdeltaE,    0    ;
           0   ,    0    ;
           0   ,    0    ];

x_dot_lin = A_lin*x_lin + B_lin*u_lin;
end

