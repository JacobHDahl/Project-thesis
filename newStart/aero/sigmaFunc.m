function sigma = sigmaFunc(alpha, M, alpha0)
    numerator = 1 + exp(-M*(alpha-alpha0)) + exp(M*(alpha+alpha0));
    denominator = (1 + exp(-M*(alpha-alpha0)))*(1 + exp(M*(alpha+alpha0)));
    sigma = numerator/denominator;
end

