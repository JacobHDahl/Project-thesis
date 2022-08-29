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

% Create data and 2-by-1 tiled chart layout
t = tiledlayout(2,1);

% Top plot
ax1 = nexttile;
plot(ax1,alphas,CLs)
hold on
plot(ax1,[-23.9,-23.9],[-2,2.8] ,'--r')
hold on
plot(ax1,[23.6,23.6],[-2,2.8],'--r')
title(ax1,'Coefficient of Lift(CL) vs. Angle of attack (alpha)')
ylabel(ax1,'CL')
xtickVals = unique([-100 : 50 : 100, alphas(Imin), alphas(Imax)]);
xtickLabs = compose('%.3g',xtickVals);
xtickLabs(ismembertol(xtickVals,[alphas(Imin), alphas(Imax)])) = {'-23.9','23.6'}; 
set(gca,'xtick', xtickVals, 'xticklabel', xtickLabs, 'xlim', [min(xtickVals),max(xtickVals)])

% Bottom plot
ax2 = nexttile;
plot(ax2,alphas,CDs)
hold on
plot(ax2,[-23.9,-23.9],[0,1],'--r')
hold on
plot(ax2,[23.6,23.6],[0,1],'--r')
title(ax2,'Coefficient of Drag(CD) vs. Angle of attack(alpha)')
ylabel(ax2,'CD')

xtickVals = unique([-100 : 50 : 100, alphas(Imin), alphas(Imax)]);
xtickLabs = compose('%.3g',xtickVals);
xtickLabs(ismembertol(xtickVals,[alphas(Imin), alphas(Imax)])) = {'-23.9','23.6'}; 
set(gca,'xtick', xtickVals, 'xticklabel', xtickLabs, 'xlim', [min(xtickVals),max(xtickVals)])
xlabel("alpha[deg]")

exportgraphics(t, "AoAvsCLCD.eps")