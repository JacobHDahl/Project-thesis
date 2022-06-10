function [deltaT, deltaE] = ezPID(eta, nu, ConstStruct, Va_target, height_target)
deltaE_trim = ConstStruct.deltaE_trim;
deltaT_trim = ConstStruct.deltaT_trim;

KpE = -0.3;
KdE = 0.2;

height_error = height_target - eta(2);

deltaE = deltaE_trim + 0.1*KpE*height_error + KdE*nu(3);
if deltaE > 1
    deltaE = 1;
elseif deltaE<-1
    deltaE = -1;

end

KpT = 0;

deltaT = deltaT_trim + KpT*height_error;

if deltaT > 1
    deltaT = 1;

elseif deltaT < 0 
    deltaT = 0;

end

end

