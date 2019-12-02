function [mat2]=get_number_mat(Nx,Ny)
%% GET THE SIZE
s1=2*Nx+1;
s2=2*Ny+1;
%% GET A BASIC MATRIX AND FIND ALL THE NODE CENTRES
mat2=ones(s1,s2);
y=(2:2:s2);
x=(2:2:s1);
[X,Y]=meshgrid(x,y);
i=sub2ind([s1 s2],X(:)',Y(:)');
%% CANCEL ALL NODE CENTRES AND GIVE OUT THE RESULTANT MATRIX
mat2(i)=0;
mat2(mat2==1)=(1:s1*s2-Nx*Ny);
mat2=mat2';
end    