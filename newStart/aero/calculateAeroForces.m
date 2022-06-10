function [FAERO_X, FAERO_Z, M_aero] = calculateAeroForces(nu,ConstStruct,deltaE,linear)
c = ConstStruct.c;
rho = ConstStruct.rho;
S = ConstStruct.S;
CL0 = ConstStruct.CL0;
CL_alpha = ConstStruct.CL_alpha;
CL_q = ConstStruct.CL_q;
CL_deltaE = ConstStruct.CL_deltaE;

CD0 = ConstStruct.CD0;
CD_alpha = ConstStruct.CD_alpha;
CD_q = ConstStruct.CD_q;
CD_deltaE = ConstStruct.CD_deltaE;

CM0 = ConstStruct.CM0;
CM_alpha = ConstStruct.CM_alpha;
CM_q = ConstStruct.CM_q;
CM_deltaE = ConstStruct.CM_deltaE;


u = nu(1);
w = nu(2);
q = nu(3);

Va = sqrt(u^2 + w^2);
alpha = atan2(w,u);

if linear
    CL = CL0 + CL_alpha*alpha + CL_q*0.5*c*q/Va + CL_deltaE*deltaE;
    CD = CD0 + CD_alpha*alpha + CD_q*0.5*c*q/Va + CD_deltaE*deltaE;
    CM = CM0 + CM_alpha*alpha + CM_q*0.5*c*q/Va + CM_deltaE*deltaE;

    F_lift = 0.5*rho*Va*Va*S*CL;
    F_drag = 0.5*rho*Va*Va*S*CD;
    M_aero = 0.5*rho*Va*Va*S*c*CM;

else
    
    [CL_ofAlpha, CD_ofAlpha] = calculateLiftDragCoeffs(alpha,ConstStruct);

    CL = CL_ofAlpha + CL_q*0.5*c*q/Va + CL_deltaE*deltaE;
    CD = CD_ofAlpha + CD_q*0.5*c*q/Va + CL_deltaE*deltaE;
    CM = CM0 + CM_alpha*alpha + CM_q*0.5*c*q/Va + CM_deltaE*deltaE;

    F_lift = 0.5*rho*Va*Va*S*CL;
    F_drag = 0.5*rho*Va*Va*S*CD;
    M_aero = 0.5*rho*Va*Va*S*c*CM;

end

R_stab2body = [cos(alpha), -sin(alpha);
                sin(alpha), cos(alpha)];

FAERO = R_stab2body*[-F_drag;-F_lift];

FAERO_X = FAERO(1);
FAERO_Z = FAERO(2);


end

