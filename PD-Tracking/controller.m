
function u = controller(params, t, x, xd)
  % x = current position
  % xd = current velocity

  % Use params.traj(t) to get the reference trajectory
  % e.g. (x - params.traj(t)) represents the instaneous trajectory error
    
  % params can be initialized in the initParams function, which is called before the simulation starts
  if t<=0
    params1=x;
  else
    params1=params.traj(t);  
  end    
  kp=1000;
  kd=100;
  
  % SOLUTION GOES HERE -------------
  u = (-0.5*kp*((x-params1)))-0.5*kd*(xd);
end