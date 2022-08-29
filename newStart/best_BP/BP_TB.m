function[net]=BP_TB(x,y,desired_error,Learning_Rate,hidden_layers,plotting)
%***
% Note : the concepts and notations that we used To name our variables are
% token from the attached PDF file of :
% R. Rojas: Neural Networks, Springer-Verlag, Berlin, 1996
%***
% BP_TB : this function allows us to train a miltilayer perceptron based on
% back propagation of the gradient "for regression".
% x : is a single matrix of M input instances by N features.
% y : is a single matrix of M input Targets by L features.
% desired_error : is a user parameter that satisfy his stop_training
% discision for the first instance.
% Learning_Rate : is a user parametrs that defines the training error
% starting from the second instance
% hidden_layers : a single raw that defines the network architecture ex:
% [10 2] = the network contains two  hidden layers of 10 neurons and 2
% neurons Consecutively.
% plotting : 'yes' if the user needs to plot the error behavior , any
% value Otherwise .
% net: contains the most important caracteristics of the network

%%%%    Authors:        TAREK BERGHOUT
%%%%    UNIVERSITY:     BATNA 2,BATNA, ALGERIA
%%%%    EMAIL:          berghouttarek@gmail.com
%%%%    Updated: 31/08/2019

% initialize parameters of training
s1=x;% inputs
s2=y;% targets
N1=max(max(s2));     % save these points for denormalizing purposes
N2=min(min(s2));     % save these points for denormalizing purposes
s1=scaledata(s1,0,1);% input  data scalling
s2=scaledata(s2,0,1);% output data scalling
% define the network architecture
network_architecture=[(size(s1,2)) hidden_layers (size(s2,2))];
% find the number of weights matrices
Number_W=length(network_architecture)-1;
ittE=[];% the error history of  the first instance
insE=[];% the behavior of error function vs all samples of the trainig set

% Training
% first step : initialization of the network
% a. generate random weights
[weights]=generateW(network_architecture,Number_W);
h = waitbar(0,'... Training process  ...');% waite bar

for samples=1:(size(s1,1))% number of instances
    waitbar(samples/ (size(s1,1)));% wait bar function
    %%%%%%%%%%%%%%%%%
    p = randperm((size(s1,1)),1);% randomly take one sample per each time
    input=s1(p,:);               % taking one instance each time
    input2=input;                % store  a copy
    output=s2(p,:);              % load one target each time
    %*** important Note (C1): the desired error must be achived only in the
    %*** first ittiration to intialize the network and after that
    %*** we could only move with biger error
    %*** generaly in the most of papers researchers uses a learning rate =0.1
    %*** starting from the second iteration
    % condition 1 :
    if samples==1 % C1
        E=desired_error;
    else
        E=Learning_Rate;
    end
    % end condition 1 :
    error=10;% initialize the error
    while error>E
        % b. calculate hidden layers (forward propagation)
        hidden=struct('F',0);% initialize hidden
        for i=1:Number_W
            W=weights(i).F;     % load weights
            H=sigmoid(input*W); % calculate the hidden layer
            hidden(i).F=H;      % save hidden layers
            input=H;            % set the actual hidden layers as the input of the next layer
                end
                % c. calculate the error
                Target_t=H;% Target of the first iteration
                error=sqrt(mse(output-Target_t));% RMSE
                % d.update weights if the error doesn't satisfy our coditions (C2)
                % d.1. calculate traversing_value of each  hidden layer (backword propagation)
                if error> E% C2
                    Trans_V=struct('F',0); % initialize traversing_value variable.
                    %%%% backpropagation starts from here%%%%
                    for i=1:Number_W

                        if i==1
                            derivative=(Target_t.*(1-Target_t));  % derivative of the activation function
                            % if you changed the activation function you have to replace this one also.
                            Trans_V(Number_W).F=(output-Target_t).*derivative; % compute & Store The traversing_value
                        else
                            H= hidden(Number_W-i+1).F;  % load the (n,n-1,n-2,...n-(number of weights matrices))th hidden layer
                            % where n is the number of hidden layers
                            derivative=(H.*(1-H));      % calculate the derivative of the hidden layer
                            sg=Trans_V(Number_W-i+2).F; % load the (m,m-1,m-2,...m-(number of weights matrices))th traversing_value
                            w1=weights(Number_W-i+2).F; % load old weights
                            % where m=n+1;
                            traversing_value=derivative.*(sg*w1');% compute The traversing value
                            % traversing_value, is in the page 8 in the attached PDF file the laste
                            % paragraphe

                            Trans_V(Number_W-i+1).F=traversing_value;% Store The traversing values
                        end
                    end
                    %%%% backpropagation ends here%%%%
                    % d.2.weights correction
                    for i=1:Number_W
                        W=weights(i).F;  % load old weights
                        H=Trans_V(i).F;  % load the appropreate traversing value
                        if i==1
                            W=W+(input2'*H); % update weights step1
                        else
                            Z=hidden(i-1).F; % load the appropreate hidden layer
                            W=W+(Z'*H);      % update weights step2
                        end
                        weights(i).F=W;  % store the new weights
                    end
                end
                if samples==1
                    ittE=[ittE error];% store error history of the first instance
                end
                input=input2;     % reload the original input to repeat the correction process
                % of the weights
    end
    insE=[insE error];% store error history of each instance
end
close(h);


if plotting=='yes'
    subplot(211)
    plot(ittE);% plotting error history of the  Forward propagation of the first instance
    xlabel('iterations of the first instance');
    ylabel('errors in the first instance');
    grid
    subplot(212)
    plot(insE);% plotting errors behaior vs training instances
    xlabel(' instances');
    ylabel('errors functin behavior');
    grid
end
% e.estimate the network training accuracy
% e.1. calculate the estimated outputs
for i=1:Number_W
    W=weights(i).F;            % load  weights
    Hidden_layer=sigmoid(s1*W);% calculate the hidden layer
    s1=Hidden_layer;           % set the hidden as the input of next hidden layer
end
s2_hat=Hidden_layer;
% e.2. calculate the Training_Accuracy
Training_Accuracy=sqrt(mse(s2_hat-s2));% root mean squered error
net.acc=Training_Accuracy;             % save training accuracy
net.IW=weights;                        % save weights
net.den=[N1 N2];                       % save denormalization parameters
net.NW=Number_W;                       % number of weight matrices
end