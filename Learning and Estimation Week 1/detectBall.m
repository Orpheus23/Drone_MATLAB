% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
%mu = rand
%sig = 
%thre = 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
%{
    I = rgb2hsv(I);
    H = I(:,:,1);
    S = I(:,:,2);
    V = I(:,:,3);
    
    N=120*160;
    x=reshape(S,1,N);
    mu=sum(x)/N;
    sigma=sqrt(sum((x-mu).^2)/N);
    pH=gauss1D(x,mu,sigma);
    pH2=reshape(pH,120,160);
    %imshow(pH2)
    %pH2(pH2>=0.3)=0;
    %pH2(pH2<0.3)=1
    pH3=pH2<=10^-3;
    %imshow(pH3)
    segI=pH3;
    
    bw_biggest = false(size(pH3));
    CC=bwconncomp(pH3);
    numPixels = cellfun(@numel,CC.PixelIdxList);
    [~,idx] = max(numPixels);
    bw_biggest(CC.PixelIdxList{idx}) = true; 
    S = regionprops(CC,'Centroid');
    imshow(bw_biggest)
    loc = S(idx).Centroid;
%}


mu = double([107.3012;124.2572;100.4691]);

sig =([58.1510 16.6644 -46.5694;16.6644 13.2688 -20.4150;-46.5694 -20.4150 66.2185]);

x = [];

for i =1:120

for j = 1:160

x = double([I(i,j,1);I(i,j,2);I(i,j,3)]);

P = exp(-0.5 .* (transpose(x-mu)* ((sig)^-1)*(x-mu)));

%P = (1/((2*3.14)^(3/2) *det(sig)^(1/2)))*exp(-0.5 * transpose(x-mu) * ((sig)^-1) * (x-mu)) ;

if P>=0.1

I(i,j,:)=255;

else

I(i,j,:) = 0;

end

end

end
    pH3=I(:,:,2);
    bw_biggest = false(size(pH3));
    CC=bwconncomp(pH3);
    numPixels = cellfun(@numel,CC.PixelIdxList);
    [~,idx] = max(numPixels);
    bw_biggest(CC.PixelIdxList{idx}) = true; 
    S = regionprops(CC,'Centroid');
    
    imshow(bw_biggest)
    loc = S(idx).Centroid;
segI=pH3;
%{
    
    H = I(:,:,1);
    S = I(:,:,2);
    V = I(:,:,3);    
    N=120*160;
    h=reshape(H,1,N);
    s=reshape(S,1,N);
    v=reshape(V,1,N);
    x=double([h;s;v]);
    mu=sum(x,2)/N;
    eps=zeros(3,3);
    for i=1:N
        eps=eps+(((x(:,i)-mu)*((x(:,i)-mu)')));
    end    
    eps=eps./N
    
p=zeros(1,N);
    for i=1:N
        P=gauss3D(x(:,i),mu,eps);
        if P<=0.1

            p(i)=255;

       
        end
    end
    
    
    outx=reshape(p,120,160);
    outx(p>=0.1)=255;
    outx(p<0.1)=0;
    pH3=outx;
    bw_biggest = false(size(pH3));
    CC=bwconncomp(pH3);
    numPixels = cellfun(@numel,CC.PixelIdxList);
    [~,idx] = max(numPixels);
    bw_biggest(CC.PixelIdxList{idx}) = true; 
    S = regionprops(CC,'Centroid');
    
    imshow(bw_biggest)
    loc = S(idx).Centroid;
segI=pH3;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.
%}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

% segI = 
% loc = 
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
