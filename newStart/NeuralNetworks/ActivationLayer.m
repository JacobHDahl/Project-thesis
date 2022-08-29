classdef ActivationLayer < Layer
    properties
        activation
        activation_prime
    end
    
    methods
        function o = ActivationLayer(activation, activation_prime)
            o.activation = activation;
            o.activation_prime = activation_prime;
        end
        
        function [o, output] = forward_propagation(o, input_data)
            o.input = input_data;
            o.output = o.activation(o.input);
            output = o.output;
        end

        function [o, activation_primed] = backward_propagation(o, output_error, learning_rate)
            activation_primed = o.activation_prime(o.input) .* output_error;
        end
    end
end

