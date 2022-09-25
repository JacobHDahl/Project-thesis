clear
clc
siz = 200;
timesteps = 25;
t = 1:siz;
sin_wave = sin(t);

x = zeros(1,timesteps);
y = zeros(1,1);
for i = 1:(siz - timesteps)
    x(:,:,i) = sin_wave(i:i+timesteps-1);
    y(:,:,i) = sin_wave(i+timesteps);
end

xt = zeros(1,timesteps);
yt = zeros(1,1);
cos_wave = cos(t);
for i = 1:(siz - timesteps)
    xt(:,:,i) = cos_wave(i:i+timesteps-1);
    yt(:,:,i) = cos_wave(i+timesteps);
end

rnn = RNNetwork(x,y,100);
rnn = rnn.train(25, 0.01, false);
rnn = rnn.test(xt,yt);

plot(rnn.outputs(1:length(rnn.outputs)/2))
hold on
plot(cos_wave(1:length(cos_wave)/2))
legend("Rnn", "GT")
hold off