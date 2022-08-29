function [W]=generateW(network_architecture,Number_W)
% this function can help  to generate random weights according 
% to the user spicified Neural Networke architecture
% network_architecture: is a vector contains  firstly the number of input
% neurons then the nember of neurons in each layer and final the number of
% output neurons of the entiere network.
% Number_W : number of weights matices in the Neural Network.
W=struct('F',0);%initialize W
for i=1:Number_W
V=rand(network_architecture(i),network_architecture(i+1));%generate waights
W(i).F=V;% save weights
end
end