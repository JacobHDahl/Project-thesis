classdef Network
    properties
        layers
        loss
        loss_prime
    end
    
    methods
        function o = Network()
            o.layers = {};
            o.loss = NaN;
            o.loss_prime = NaN;
        end
        function o = add(o, layer)
            o.layers{end+1} = layer;
        end
        function o = use(o, loss, loss_prime)
            o.loss = loss;
            o.loss_prime = loss_prime;
        end
        function [o, result] = predict(o, input_data)
            samples = length(input_data);
            result = {};

            for i = 1:samples
                output = input_data(i,:)';
                for j = 1:length(o.layers)
                    layer = o.layers{j};
                    [layer, output] = layer.forward_propagation(output);
                    o.layers{j} = layer;
                end
                result{end+1} = output;
    
            end
        end

        function o = fit(o, x_train, y_train, epochs, learning_rate)
            samples = length(x_train);

            for i = 1:epochs
                err = 0;
                for j = 1:samples
                    output = x_train(j,:)';
                    for k = 1:length(o.layers)
                        layer = o.layers{k};
                        [layer, output] = layer.forward_propagation(output);
                        o.layers{k} = layer;
                    end
                    
                    err = err + o.loss(y_train(j), output);

                    error = o.loss_prime(y_train(j), output);

                    for k = length(o.layers):-1:1
                        layer = o.layers{k};
                        [layer, error] = layer.backward_propagation(error, learning_rate);
                        o.layers{k} = layer;
                    end
                end
                err = err / samples;
                disp(['epoch: ', num2str(i), '/', num2str(epochs), 'error: ', num2str(err)]);
            end
        end
    end
end

