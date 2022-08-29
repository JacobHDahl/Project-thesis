clear
clc
activation_prime = @(x) 1-tanh(x).*tanh(x);
activation = @tanh;

mse = @(y_true, y_pred) mean((y_true - y_pred).^2);
mse_prime = @(y_true, y_pred) 2*(y_pred-y_true)/length(y_true);

net = Network();
FC1 = FCLayer(2,5);
net = net.add(FC1);
AL1 = ActivationLayer(activation, activation_prime);
net = net.add(AL1);
FC2 = FCLayer(5,3);
net = net.add(FC2);
AL2 = ActivationLayer(activation, activation_prime);
net = net.add(AL2);
FC2 = FCLayer(3,1);
net = net.add(FC2);
AL3 = ActivationLayer(activation, activation_prime);
net = net.add(AL3);

net = net.use(mse, mse_prime);
x_train = [0,0; 0,1; 1,0; 1,1];
y_train = [0;1;1;0];
net = net.fit(x_train, y_train, 1000, 0.1);

[net, out] = net.predict(x_train);
disp(out)


