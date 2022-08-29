timeStruct = load("t.mat");
t = timeStruct.t;

heightRef = load("heightTargetDataFlying.mat");
heightRef_data = heightRef.heightTargetData;

heightData = load("heightDataFlying.mat");
heightData_data = heightData.heightData;

heightDataPID = load("heightDataFlyingPID.mat");
heightDataPID_data = heightDataPID.heightDataPID;

fig = figure(1);

plot(t,heightRef_data(1:end),'LineWidth',1);
hold on
plot(t,heightData_data(1:end-1),'LineWidth',1);
hold on
plot(t,heightDataPID_data(1:end-1),'LineWidth',1);
xlabel('Time[s]')
ylabel('Height[m]')
legend('Target','Dynamic Inv.','PID')
legend('Location','best')


