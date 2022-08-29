trainFcn = 'trainlm';
hiddenLayerSize=8;
net = feedforwardnet(hiddenLayerSize,trainFcn);

[x,t] = simplefit_dataset;

net.trainParam.showWindow=0;
net.trainParam.epochs=1;
net = train(net,x,t);


weights= getwb(net);

wts = net.LW;
wts2 = net.IW;


net2 = feedforwardnet(hiddenLayerSize,trainFcn);
net2.initFcn = 'initlay';
net2.layers{1}.initFcn = 'initwb';


