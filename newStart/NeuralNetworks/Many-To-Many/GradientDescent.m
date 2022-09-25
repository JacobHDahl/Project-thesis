classdef GradientDescent
    properties
        beta
        lr
    end
    
    methods
        function o = GradientDescent(beta, lr)
            o.beta = beta;
            o.lr = lr;
        end
        
        function [o, new_weights, velocities] = optim(o, weights, gradients, velocities)
            if isnan(velocities)
                velocities = zeros(1, length(weights(1,:)));
            end
            velocities = o.update_velocities(gradients, o.beta, velocities);
            new_weights = {};

            for i = 1:length(weights)
                weight = weights{i};
                velocity = velocities(i);
                weight = weight - o.lr * velocity;
                new_weights{end+1} = weight;
            end
        end

        function velocities = update_velocities(o,gradients, beta, velocities)
            new_velocities = {};
            for i = 1:length(gradients)
                gradient = gradients{i};
                velocity = velocities(i);

                new_velocity = beta * velocity + (1 - beta) * gradient;
                new_velocities{end+1} = new_velocity;
            end
        end
    end
end

