function z = MVN_SampleGen(mu,sigma,N)
    %Computes Samples for a MVN PDF
    z = mu + chol(sigma).'*randn(length(mu),N);
end