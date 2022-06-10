addpath(genpath("..\"));
ConstStruct = load("ConstFile.mat");
alphas = -90:0.1:90;

CLs = zeros(1,length(alphas));
CDs = zeros(1,length(alphas));

for i = 1:length(alphas)
    alpha = deg2rad(alphas(i));
    [CL,CD] = calculateLiftDragCoeffs(alpha,ConstStruct);
    CLs(i) = CL;
    CDs(i) = CD;

end

figure(1)
plot(alphas,CLs)
xlabel("alpha[deg]")
ylabel("CL")

figure(2)
plot(alphas,CDs)
xlabel("alpha[deg]")
ylabel("CD")