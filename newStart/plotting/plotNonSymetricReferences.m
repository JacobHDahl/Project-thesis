fig = figure(1);

nonSym = load("nonSymData2.mat");
nonSym_data = nonSym.data;
nonSymRef = load("nonSymRef2.mat");
nonSymRef_data = nonSymRef.dataref;

plot(t,nonSymRef_data(1:end-1),'-','LineWidth',1)
hold on
plot(t,nonSym_data(1:end-1),'-','LineWidth',1)

xlabel("Time[s]");
ylabel("Pitch rate[rad/s]")
legend('Ref','Pitch rate')

hold off
exportgraphics(fig, "NonSymmetricReference.eps")