fig = figure(1);

randomData = load("randomData.mat");
randomData_data = randomData.data;
randomRef = load("randomRef.mat");
randomRef_data = randomRef.dataref;

shiftedData = load("randomData2.mat");
shiftedData_data = shiftedData.data;



plot(t,randomRef_data(1:end-1),'-.','LineWidth',1)
hold on
plot(t,randomData_data(1:end-1),'-','LineWidth',1)
hold on
plot(t,shiftedData_data(1:end-1),'-','LineWidth',1)

xlabel("Time[s]");
ylabel("Pitch rate[rad/s]")
legend('Ref','Pitch rate', 'Pitch rate perturbed')

hold off
exportgraphics(fig, "randomReferencePerturbed.eps")