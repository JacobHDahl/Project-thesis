clear all ;
clc;
%% Load data
data=load('simplefit_dataset');
x=data.simplefitInputs'; 
y=data.simplefitTargets';
%% Initialize parameters
desired_error=0.1;
Learning_Rate=0.1;
hidden_layers=[5];
plotting='yes';
%% Training
[net]=BP_TB(x,y,desired_error,Learning_Rate,hidden_layers,plotting);
%%%%%%%%%%% prediction
%% Prediction 
[outputs]=predict(net,x);
%%% Illustration
figure(2)
plot(x,y,'+-r',x,outputs,'+-b');
legend('original values','predicted values');
grid
