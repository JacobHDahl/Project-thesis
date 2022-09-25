num_epochs = 1001;
input_dim = 5;
output_dim = 5;
hidden_dim = 15;

% initialize and define the model hyperparamaters
model = RNN(input_dim, output_dim, hidden_dim);
beta = 0.9;
learning_rate = 0.01;
optim = GradientDescent(beta, learning_rate);
costs = [];

t = 1:1000000;
gt = sin(t);

epochs = round(length(t) / output_dim);

for i = 1:epochs

    try
        X = num2cell(t(i*5:i*5+4));
        Y = num2cell(gt(i*5:i*5+4));
    catch
        break;
    end
    
    [model, y_preds] = model.forward(X);
    [model, costs] = model.loss(Y);
    model = model.backward();
    model = model.clip(1);
    model = model.optimize(optim);
    disp(["Cost:", sum(costs)])
end

