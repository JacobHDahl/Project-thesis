classdef (Abstract) Layer
    properties
        input
        output
    end
    
    methods
        forward_propagation(o, input)
        backward_propagation(o, output_error, learning_rate)
    end
end

