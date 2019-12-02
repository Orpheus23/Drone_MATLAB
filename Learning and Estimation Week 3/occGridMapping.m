% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function map = occGridMapping(ranges, Angles, pose, param)



% Parameters 
% 
% % the number of grids for 1 meter.
 Resol = param.resol;
% % the initial map size in pixels
 map = zeros(param.size);
% % the origin of the map in pixels
 myorigin = param.origin; 
% % 4. Log-odd parameters 
 lo_occ = param.lo_occ;
 lo_free = param.lo_free; 
 lo_max = param.lo_max;
 lo_min = param.lo_min;
 N = size(pose,2);
 for j = 1:N 
% Get the basic params
state=[pose(1,j);pose(2,j)];
theta=pose(3,j)';
angle_main=(theta+Angles');
d=ranges(:,j)';
% Find grids hit by the rays (in the gird map coordinate)
r1=d.*cos(angle_main); 
r2=-d.*sin(angle_main);
p=[r1;r2]+state;
i_occ=ceil(Resol.*p)+myorigin;

% Find occupied-measurement cells and free-measurement cells
for i = 1:size(i_occ,2)
orig = [ceil(Resol*pose(1,j))+myorigin(1),ceil(Resol*pose(2,j))+myorigin(2)]; % start point
[freex, freey] = bresenham(orig(1),orig(2),i_occ(1,i),i_occ(2,i));  

% convert to 1d index
free = sub2ind(size(map),freey,freex);

% set end point value 
%map(i_occ(2,i),i_occ(1,i)) = 1;
% set free cell values
occ = sub2ind(size(map),i_occ(2,i),i_occ(1,i));
% Saturate the log-odd values
  
  map(occ) = map(occ)+lo_occ;
  map(free) = map(free)-lo_free;
  
  
  if (map(occ)>=lo_max)
      map(occ)=lo_max;
  end
  if(map(free)<=lo_min)
      map(free)=lo_min;
  end
end
% Update the log-odds


 
% Visualize the map as needed

% 
 end

end

