function zk = z (x,mu,sigma)
zk=[];
for i=1:3
    zk=[zk;gauss3D(x(:,i),mu(:,i),sigma(:,:,i))];
end
zk=zk./(sum(zk));
end