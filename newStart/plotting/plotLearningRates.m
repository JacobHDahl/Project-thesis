timeStruct = load("t.mat");
fiftyOne = load("50-1BESTWeights.mat");
fiftyFifty = load("50-50Weights.mat");
oneOne = load("1-1Weights.mat");
tenTen = load("10-10Weights.mat");
hundredHundred = load("100-100Weights.mat");
refStruct = load("ref.mat");

t = timeStruct.t;
fiftyOne_data = fiftyOne.data;
fiftyFifty_data = fiftyFifty.data;
oneOne_data = oneOne.data;
tenTen_data = tenTen.data;
hundredHundred_data = hundredHundred.data;

ref = refStruct.data;

line = 0.1;

fig = figure(1);
plot(t,ref(1:end-1),'LineWidth',1)
hold on
plot(t,fiftyOne_data(1:end-1),'-','LineWidth',1)
hold on
% plot(t,fiftyFifty_data(1:end-1),'-','LineWidth',1); 
% hold on
plot(t,oneOne_data(1:end-1),'-','LineWidth',1)
hold on
% plot(t,tenTen_data(1:end-1),'-','LineWidth',1)
% hold on
plot(t,hundredHundred_data(1:end-1),'-.','LineWidth',0.001)
% p = plot(t,nonAdapted(1:end-1),'--','LineWidth',1);
% p.Color = 	'#77AC30';
% hold on

xlabel('Time[s]')
ylabel('Pitch rate[rad/s]')
legend('Ref','50-1','1-1','100-100')
hold off
exportgraphics(fig, "DifferentLearningRates2.eps")