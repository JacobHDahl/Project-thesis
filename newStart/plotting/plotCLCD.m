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

[ymin, Imin] = min(CLs);
[ymax, Imax] = max(CLs);

t=figure(1);
plot(alphas,CLs)
hold on
plot([-23.9,-23.9],[-2,2.5],'b' ,'--')
hold on
plot([23.6,23.6],[-2,2.5],'b','--')
%set(gca,'xtick', [alphas(Imin), alphas(Imax)], 'xticklabel', {'f_1','f_2'}, 'xlim', [-100,100])

xtickVals = unique([-100 : 50 : 100, alphas(Imin), alphas(Imax)]);
xtickLabs = compose('%.3g',xtickVals);
xtickLabs(ismembertol(xtickVals,[alphas(Imin), alphas(Imax)])) = {'-23.9','23.6'}; 
set(gca,'xtick', xtickVals, 'xticklabel', xtickLabs, 'xlim', [min(xtickVals),max(xtickVals)])
% hold on
% plot(alphas(xmin),ymin,'.','MarkerSize',10)
% hold on
% plot(alphas(xmax),ymax,'.','MarkerSize',10)
xlabel("alpha[deg]")
ylabel("CL")
title("Coefficient of Lift (CL) against Angle of attack (alpha)")
exportgraphics(t, "AoAvsCL.eps")

figure(2)
plot(alphas,CDs)
xlabel("alpha[deg]")
ylabel("CD")