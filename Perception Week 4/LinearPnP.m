function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 1) pose rotation
%
% IMPORTANT NOTE: While theoretically you can use the x directly when solving
% for the P = [R t] matrix then use the K matrix to correct the error, this is
% more numeically unstable, and thus it is better to calibrate the x values
% before the computation of P then extract R and t directly
%{
N=size(X,1);
%{
x2=[x ones(N,1)];
x2=inv(K)*x2';
x2=x2';
x2=x2./x2(:,3);
%xc=xc(:,1:2);
%}
x2=((K\[x';ones(1,size(x,1))]))'; 
a=[];
for i=1:N
A=zeros(3,12);
A(1,1:4)=[X(i,:) 1];
A(2,5:8)=[X(i,:) 1];
A(3,9:12)=[X(i,:) 1];
Ak=Skew((x2(i,:)))*A;
a=[a;Ak];
end
[~,~,v]=svd(a);
P=v(:,end);
P=reshape(P,4,3)';
R1=P(:,1:3);

[u,d,v]=svd(R1);
s = det(u * v');
R = s * u * v';
t = s * P(:,4)/d(1);
R=det(R)*R;
C=-R'*t;
end
function S=Skew(x)
S=[0 -x(3) x(2);x(3) 0 -x(1);-x(2) x(1) 0];
%}
num_points = size(X,1);
A = []; % numpoints*3x12

for i = 1:num_points
    
    X_tilde = [X(i,:) 1]';    
    x_i = [x(i,:) 1]';
    
    x_i = inv(K)*x_i;
    
    M = [X_tilde' zeros(1,4) zeros(1,4);...
        zeros(1,4) X_tilde' zeros(1,4);...
        zeros(1,4) zeros(1,4) X_tilde'];
    
    A_i = Vec2Skew(x_i)*M;
    
    A = [A;A_i];
end

[~,~,v]=svd(A);
p=v(:,end);
p=reshape(p,4,3);
p=transpose(p);
R=p(:,1:3);
t=p(:,4);
[u,d,v]=svd(R);
%R=u*[1 0 0;0 1 0;0 0 det(u*v)]*v;
s = det(u * v');
R = s * u * v';
t = s * p(:,4) / d(1,1);
C=-transpose(R)*t;
end






