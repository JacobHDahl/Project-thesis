classdef RNNetwork
    properties
        x
        y
        Wx
        Wh
        Wy
        by
        b

        hidden_units
        hidden_states
        inputs
        outputs
        error
        loss
        yt
        lr
        Ovr_loss
    end
    
    methods
        function o = RNNetwork(x, y, hidden_units)
            o.x = x;
            o.y = y;
            o.hidden_units = hidden_units;

            o.Wx = randn(o.hidden_units, length(o.x(:,1,1)));
            o.Wh = randn(o.hidden_units, o.hidden_units);
            o.Wy = randn(length(o.y(:,1,1)), o.hidden_units);
            o.b = randn(hidden_units, 1);
            o.by = randn(1);

            o.error = 0;
            o.loss = 0;
        end
        
        function [ht, yt] = cell(o, xt, ht_1)
            ht = tanh(o.Wx * xt + o.Wh * ht_1 + o.b);
            yt = o.Wy * ht + o.by;
        end

        function o = forward(o, sample)
            sample_x = o.x(sample);
            sample_y = o.y(sample);

            ht = zeros(o.hidden_units, 1);
            o.hidden_states = {ht};
            o.inputs = {};
            for step = 1:length(sample_x)
                [ht, yt] = o.cell(sample_x(step), ht);
                o.inputs{end+1} = sample_x(step);
                o.hidden_states{end+1} = ht;
            end
            o.error = yt - sample_y;
            o.loss = 0.5*o.error*o.error;
            o.yt = yt;
        end

        function o = backward(o)
            n = length(o.inputs);
            dyt = o.error;
            dWy = dyt * o.hidden_states{end}';
            dht = (dyt * o.Wy)';
            dWx = zeros(size(o.Wx));
            dWh = zeros(size(o.Wh));
            dby = zeros(size(o.by));
            db = zeros(size(o.b));

            %BPTT
            for step = n:-1:1
                temp = (1-o.hidden_states{step+1}.^2).*dht;
                dWx = dWx + temp * o.inputs{step}';
                dWh = dWh + temp + o.hidden_states{step}';
                dby = dby + dyt;
                db = db + (1-o.hidden_states{step+1}.^2);

                dht = o.Wh * temp;
            end
            dWy(dWy > 1) = 1;
            dWy(dWy < -1) = -1;
            dWx(dWx > 1) = 1;
            dWx(dWx < -1) = 1;
            dWh(dWh > 1) = 1;
            dWh(dWh < -1) = 1;

            o.Wy = o.Wy - o.lr*dWy;
            o.Wx = o.Wx - o.lr*dWx;
            o.Wh = o.Wh - o.lr*dWh;
        end

        function o = train(o, epochs, learning_rate, debug)
            o.Ovr_loss = [];
            o.lr = learning_rate;
            for epoch = 1:epochs
                for sample = 1:length(o.x(1,1,:))
                    o = o.forward(sample);
                    o = o.backward();
                end
                o.Ovr_loss(end+1) = o.loss / length(o.x(1,:));
                if debug
                    disp(["Epoch: ", num2str(epoch), "Loss: ", num2str(mean(o.Ovr_loss))])
                end
                o.loss= 0;
            end
        end

        function o = test(o, x, y)
            o.x = x;
            o.y = y;
            o.outputs = [];
            for sample = 1:length(x)
                o = o.forward(sample);
                o.outputs(end+1) = o.yt;
            end
        end
     
    end
end

