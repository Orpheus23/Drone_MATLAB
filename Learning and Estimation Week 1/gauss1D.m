function P = gauss1D (x,mu,sigma)
P=exp((-0.5*(x-mu).^2)*(sigma^(-2)));
%(1/(sigma*((2*pi)^(1/2))))     x=[3 x 1];same for mu;sigma = [3 x 3]
end