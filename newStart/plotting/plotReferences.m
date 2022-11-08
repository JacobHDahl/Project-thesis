timeStruct = load("t.mat");
t = timeStruct.t;

fiftyOne = load("50-1BESTWeights.mat");
fiftyOne_data = fiftyOne.data;
refStruct = load("ref.mat");
ref = refStruct.data;

K_5 = load("50-10_K-5.mat");
K_5_data = K_5.data;
ref_K_5 = load("K-5REF.mat");
ref_K_5_data = ref_K_5.dataref;



K_1 = load("50-10_K-1.mat");
K_1_data = K_1.data;
ref_K_1 = load("K-1REF.mat");
ref_K_1_data = ref_K_1.dataref;

K_01 = load("50-10_K-01.mat");
K_01_data = K_01.data;
ref_K_01 = load("K-01REF.mat");
ref_K_01_data = ref_K_01.dataref;

K_05 = load("50-10_K-05.mat");
K_05_data = K_05.data;
ref_K_05 = load("K-05REF.mat");
ref_K_05_data = ref_K_05.dataref;

line = 0.1;
fig = figure(1);

subplot(5,1,1)
plot(t,ref(1:end-1),'LineWidth',1)
hold on
plot(t,fiftyOne_data(1:end-1),'-','LineWidth',1);
legend('Ref','Pitch rate')
legend('Location','best')

subplot(5,1,2)
plot(t,ref_K_1_data(1:end-1),'-','LineWidth',1)
hold on
plot(t,K_1_data(1:end-1),'-','LineWidth',1); 


subplot(5,1,3)
plot(t,ref_K_5_data(1:end-1),'-','LineWidth',1)
hold on
plot(t,K_5_data(1:end-1),'-','LineWidth',1)
ylabel('Pitch rate [rad/s]')


subplot(5,1,4)
plot(t,ref_K_01_data(1:end-1),'-','LineWidth',1)
hold on
plot(t,K_01_data(1:end-1),'-','LineWidth',1)

subplot(5,1,5)
plot(t,ref_K_05_data(1:end-1),'-','LineWidth',1)
hold on
plot(t,K_05_data(1:end-1),'-','LineWidth',1)

% p = plot(t,nonAdapted(1:end-1),'--','LineWidth',1);
% p.Color = 	'#77AC30';
% hold on

xlabel('Time[s]')

%exportgraphics(fig, "DifferentReferences.eps")