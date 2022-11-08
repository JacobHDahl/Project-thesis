timeStruct = load("t.mat");
t = timeStruct.t;
adapted = load("adapted.mat");
adapted_data = adapted.data;
nonAdapted = load("NONadapted.mat");
nonAdapted_data = nonAdapted.data;
ref = load("adaptedRef.mat");
ref_data = ref.dataref;

fig = figure(1);
plot(t,ref_data(1:end-1),'LineWidth',1)
hold on
plot(t,adapted_data(1:end-1),'-','LineWidth',1)
hold on
plot(t,nonAdapted_data(1:end-1),'-','LineWidth',1); 
% p = plot(t,nonAdapted(1:end-1),'--','LineWidth',1);
% p.Color = 	'#77AC30';
% hold on

xlabel('Time[s]')
ylabel('Pitch rate[rad/s]')
legend('Ref','Adapted','Non-adapted')
hold off
exportgraphics(fig, "AdaptedVsNonAdapted.eps")