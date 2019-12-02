function [mat2] = get_mat(Nx,Ny,constr,type)
%constraint_vector defined as
%{[top_DOF],[Bottom_DOF],[left_DOF],[right_DOF]}
%% DEFINE THE SIZE AND GET THE MATRIX %%
s1=2*Nx+1;
s2=2*Ny+1;
mat = ones(s2,s1,5);
len = s1*s2;
%% DEFINE THE SIZE TO BE CUT FOR GETTING A 8DOF ELEMENT %%
if type==8
y=(2:2:s2);
x=(2:2:s1);
[X,Y]=meshgrid(y,x);
X2=repmat(X(:),5,1)';
Y2=repmat(Y(:),5,1)';
z=reshape(repmat((1:5),size(X(:))),[1,5*size(X(:),1)]);
%Get the indices to be cut
cut=sub2ind(size(mat),X2,Y2,z);
end
%% APPLY THE BOUNDARY CONDITIONS
try
mat(1,:,cell2mat(constr(1)))=0;
catch
end    
try
mat(end,:,cell2mat(constr(2)))=0;
catch
end    
try
mat(:,1,cell2mat(constr(3)))=0;
catch
end    
try
mat(:,end,cell2mat(constr(4)))=0;
catch 
end
%% REMOVING THE UNNECESSARY ELEMENTS HERE(IF 8DOF) AND GETTING THE FINAL MATRIX %%
if type ==8
mat(cut)=-3;
len=s1*s2-Nx*Ny;   
end
seq=(1:len);
mat2=reshape(mat(mat~=-3),[len,5])';
%% GET THE NODE NUMBER MATRIX HERE BY APPENDING THE SEQUENCE TO THE DOF LIST %%
mat2(mat2==1)=(1:sum(mat2,'all'));
mat2=[seq' mat2'];
end