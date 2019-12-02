function[node] = get_element_node(Nx,Ny)
%% USED TO DEFINE ALL THE POINTS AROUND THE ELEMENT
points_around=[-1 -1;-1 1;1 1;1 -1;-1 0;0 1;1 0;0 -1];
%% DEFINE THE SIZE OF THE MATRIX
s1=2*Nx+1;
s2=2*Ny+1;
%% SUPPLIMENTARY FUNCTION TO PRESENT ALL NODES AROUND AN ELEMENT
node_mat=get_number_mat(Nx,Ny);
%% COUNT OF ALL ELEMENTS AND THEIR ORIGIN COORDINATES
y=(2:2:s2);
x=(2:2:s1);
[X,Y]=meshgrid(y,x);
elem_count=[X(:) Y(:)];
i=1;
%% PREALLOCATING NODE
node=zeros(size(elem_count,1),9);
for elem=elem_count'
    pts=(elem'+points_around)';
    a=sub2ind([s2,s1],pts(1,:),pts(2,:));
    node(i,:)=[i node_mat(a)];
    i=i+1;
end    
end