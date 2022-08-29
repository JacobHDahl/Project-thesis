classdef FCLayer < Layer
    %FCLAYER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        weights
        bias
    end
    
    methods
        function o = FCLayer(inputSize, outputSize)
            %Initialize weights and biases to random values
            o.weights = randn(outputSize, inputSize) - 0.5;
            o.bias = randn(outputSize, 1) - 0.5;
        end
        
        function [o, output] = forward_propagation(o, input_data)
            o.input = input_data;
            o.output = o.weights * o.input + o.bias;
            output = o.output;
        end

        function [o, input_error] =  backward_propagation(o, output_error, learning_rate)
            input_error = o.weights'* output_error;
            weights_error = o.input * output_error';
            %dBias = output_error

            %update parameters
           o.weights = o.weights - learning_rate * weights_error';
           o.bias = o.bias - learning_rate * output_error;

        end
    end
end

