function X = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)
%% LinearTriangulation
% Find 3D positions of the point correspondences using the relative
% position of one camera from another
% Inputs:
%     C1 - size (3 x 1) translation of the first camera pose
%     R1 - size (3 x 3) rotation of the first camera pose
%     C2 - size (3 x 1) translation of the second camera
%     R2 - size (3 x 3) rotation of the second camera pose
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Outputs: 
%     X - size (N x 3) matrix whos rows represent the 3D triangulated
%       points
N=size(x1,1);
X=zeros(N,3);
P1=K*R1*[eye(3,3) -C1];P2=K*R2*[eye(3,3) -C2];

X1=[x1';ones(1,N)];
X2=[x2';ones(1,N)];

for i=1:N
A=[Skew(X1(:,i))*P1;Skew(X2(:,i))*P2];
[~,~,v]=svd(A);

temp=v(1:3,end)/v(end,end);
X(i,:)=temp(1:3,1)';
end

end

function S=Skew(x)
S=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
end