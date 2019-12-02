function P = gauss3D (x,mu,sigma)
P=(1/(sqrt(det(sigma))*((2*3.1415)^(3/2))))*exp(-0.5.*(((x-mu)'*(sigma^-1)*(x-mu))));
%x=[3 x 1];same for mu;sigma = [3 x 3] 
end