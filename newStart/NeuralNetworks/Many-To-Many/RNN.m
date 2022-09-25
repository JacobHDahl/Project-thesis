classdef RNN
    properties
        input_dim
        output_dim
        hidden_dim
        Wya
        Wax
        Waa
        by
        b

        tanh_prime
        softmax
        oparams

        input_X
        hidden_list
        y_preds
        Y
        dWax
        dWaa
        dWya
        db
        dby
    end
    
    methods
        function o = RNN(input_dim, output_dim, hidden_dim)
            %
            %Initialize the parameters with the input, output and hidden
            %dimensions. 
            %Parameters
            %----------
            %input_dim : int
            %    Dimension of the input. 
            %output_dim : int
            %    Dimension of the output.
            %hidden_dim : int
            %    Number of units in the RNN cell.
            %
            o.input_dim = input_dim;
            o.output_dim = output_dim;
            o.hidden_dim = hidden_dim;
            
            [o.Wya, o.Wax, o.Waa, o.by, o.b] = o.initialize_parameters(input_dim, output_dim, hidden_dim);
            o.softmax = @(x) exp(x) ./ sum(exp(x));
            o.tanh_prime = @(x) (1 - tanh(x).*tanh(x));
            o.oparams = NaN;
        end

        function [o, y_preds] = forward(o, input_X)
            %"""
            %Computes the forward propagation of the RNN.
            %Parameters
            %----------
            %input_X : numpy.array or list
            %    List containing all the inputs that will be used to 
            %    propagete along the RNN cell.
            %Returns
            %-------
            %y_preds : list
            %    List containing all the preditions for each input of the
            %    input_X list.
            %"""

            o.input_X = input_X;
            hidden = zeros(o.hidden_dim, o.output_dim);
            o.hidden_list = {};
            o.hidden_list{end+1} = hidden;
            o.y_preds = {};

            for i = 1:length(input_X)
                input_x = o.input_X{i};
                input_tanh = o.Wax * input_x + o.Waa * hidden + o.b;
                hidden = tanh(input_tanh);
                o.hidden_list{end+1} = hidden;

                input_softmax = o.Wya * hidden + o.by;
                y_pred = o.softmax(input_softmax);
                o.y_preds{end+1} = y_pred;
            end

            y_preds = o.y_preds;
        end

        function [o, cost] = loss(o, Y)
            %"""
            %Computes the Cross Entropy Loss for the predicted values.
            %Parameters
            %----------
            %Y : numpy.array or list
            %    List containing the real labels to predict.
            %Returns
            %-------
            %cost : int
            %    Cost of the given model.
            %"""

            o.Y = Y;
            cost = 0;

            for i = 1:length(Y)
                y_pred = o.y_preds{i};
                y = Y{i};
                
                lossTmp = - y .* log(y_pred);
                loss = sum(lossTmp);
                cost = cost + loss;
            end
        end

        function o = backward(o)
            %"""
            %Computes the backward propagation of the model.
            %Defines and updates the gradients of the parameters to used
            %in order to actulized the weights.
            %"""
            
            [o.dWax, o.dWaa, o.dWya, o.db, o.dby, dhidden_next] = o.define_gradients();

            for i = length(o.Y):-1:1
                dy = o.y_preds{i} - o.Y{i};

                hidden = o.hidden_list{i+1};
                hidden_prev = o.hidden_list{i};

                %gradients y
                o.dWya = o.dWya + dy * hidden';
                o.dby = o.dby + dy;
                dhidden = o.Wya' * dy + dhidden_next;

                %gradients a
                dtanh = o.tanh_prime(dhidden .* o.hidden_list{i});
                o.db = o.db + dtanh;
                o.dWax = o.dWax + dtanh * o.input_X{i}';
                o.dWaa = o.dWaa + dtanh * hidden_prev';
                dhidden_next = o.Waa' * dtanh;
            end

        end

        function o = clip(o, clip_value)
            %"""
            %Clips the gradients in order to avoisd the problem of 
            %exploding gradient.
            %Parameters
            %----------
            %clip_value : int
            %    Number that will be used to clip the gradients.
            %"""
            
            if o.dWax > clip_value
                o.dWax = clip_value;
            end
            if o.dWax < -clip_value
                o.dWax = -clip_value;
            end
            if o.dWaa > clip_value
                o.dWaa = clip_value;
            end
            if o.dWaa < -clip_value
                o.dWaa = -clip_value;
            end
            if o.dWya > clip_value
                o.dWya = clip_value;
            end
            if o.dWya < -clip_value
                o.dWya = -clip_value;
            end
            if o.db < -clip_value
                o.db = -clip_value;
            end
            if o.db > clip_value
                o.db = clip_value;
            end
            if o.dby < -clip_value
                o.dby = -clip_value;
            end
            if o.dby > clip_value
                o.dby = clip_value;
            end

        end

        function o = optimize(o, method)
            %"""
            %Updates the parameters of the model using a given optimize 
            %method.
            %Parameters
            %----------
            %method: Class
            %    Method to use in order to optimize the parameters.
            %"""
            weights = {o.Wya, o.Wax, o.Waa, o.by, o.b};
            gradients = {o.dWya, o.dWax, o.dWaa, o.dby, o.db};

            [method, new_weights, o.oparams] = method.optim(weights, gradients, o.oparams);
            o.Wya = new_weights{1};
            o.Wax = new_weights{2};
            o.Waa = new_weights{3};
            o.by = new_weights{4};
            o.b = new_weights{5};
        end

        function [dWax, dWaa, dWya, db, dby, da_next] = define_gradients(o)
            %"""
            %Defines the gradients of the model.
            %"""

            dWax = zeros(size(o.Wax));
            dWaa = zeros(size(o.Waa));
            dWya = zeros(size(o.Wya));

            db = zeros(size(o.b));
            dby = zeros(size(o.by));
            
            da_next = zeros(size(o.hidden_list{1}));

            

        end
        
        function [weights_y, weights_ax, weights_aa, bias_y, bias] = initialize_parameters(o,input_dim, output_dim, hidden_dim)
            %
            %Initialize the parameters randomly.
            %Parameters
            %----------
            %input_dim : int
            %    Dimension of the input
            %output_dim : int
            %    Dimension of the ouput
            %hidden_dim : int
            %Returns
            %-------
            %weights_y : numpy.array
            %weights_ax : numpy.array
            %weights_aa : numpy.array
            %bias_y : numpy.array
            %bias : numpy.array
            %

            den = sqrt(hidden_dim);

            weights_y = randn(output_dim, hidden_dim) / den;
            bias_y = zeros(output_dim, 1);
            weights_ax = randn(hidden_dim, input_dim) / den;
            weights_aa = randn(hidden_dim, hidden_dim) / den;
            bias = zeros(hidden_dim, 1);

        end
    end
end

