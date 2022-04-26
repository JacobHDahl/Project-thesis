function blended = sigmoid_alpha(alpha, alpha_0, M)
%SIGMOID_ALPHA eq 4.10 in Beard&McLain
nominator = 1 + exp(-M*(alpha-alpha_0)) + exp(M*(alpha+alpha_0));
denominator = (1 + exp(-M*(alpha-alpha_0)))*1 + exp(M*(alpha+alpha_0));
blended = nominator/denominator;
end

