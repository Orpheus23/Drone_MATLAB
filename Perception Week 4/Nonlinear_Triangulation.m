function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     x
% Outputs: 
%     X - size (N x 3) matrix of refined point 3D locations 

%{
X1=X0;
X2= SingleP(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0);
epsilon=(10^(-10));
i=1;
while immse(X2, X1)>=epsilon
    if i==180
        X1=X0;
    break;
    end
    X1=X2;
    X2=SingleP(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X1);
    i=i+1;
end
end
%}
N=size(X0,1);
X=zeros(N,3);
for i=1:N
delxi=Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1(i,:), x2(i,:), x3(i,:), X0(i,:));
a=1;
epsilon=10^(-20);
while det(delxi*delxi')>=epsilon
    if a==480
        break
    end    
    delxi=Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1(i,:), x2(i,:), x3(i,:), X0(i,:)-delxi);
    a=a+1;
end    
X(i,:)=X0(i,:)-delxi;
end    
end

function delx = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
[F1,P1]=Jacob(R1,K,X0,C1);
[F2,P2]=Jacob(R2,K,X0,C2);
[F3,P3]=Jacob(R3,K,X0,C3);
J=[F1' F2' F3']';
fx=[P1(1)/P1(3) P1(2)/P1(3) P2(1)/P2(3) P2(2)/P2(3) P3(1)/P3(3) P3(2)/P3(3)]';
b=[x1(1) x1(2) x2(1) x2(2) x3(1) x3(2)]';
delx=inv(J'*J)*J'*(b-fx);
delx=delx';
end

function [J1,P1] = Jacob(R, K, X, C)
    out=K*R;
    dudx=out(1,:);
    dvdx=out(1,:);
    dwdx=out(1,:);
    P1=K*R*(X-C);
    %P = K*R*[eye(3) -C];
    %P1 = P*[X;1];
    J1=[(P1(3)*dudx-P1(1)*dwdx)/(P1(3)^2);(P1(3)*dvdx-P1(2)*dwdx)/(P1(3)^2)];



end
