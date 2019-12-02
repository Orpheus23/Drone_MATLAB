function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
Kd=[2.5 2.5 2.5];
Kp=[20 20 20];
kpph=1;kdph=0.06;kpth=1;kdth=0.06;kpps=1;kdps=0.06;
rddot_des1 = des_state.acc(1) + Kd(1)*(des_state.vel(1)-state.vel(1)) +Kp(1)*(des_state.pos(1)-state.pos(1));
rddot_des2 = des_state.acc(2) + Kd(2)*(des_state.vel(2)-state.vel(2)) +Kp(2)*(des_state.pos(2)-state.pos(2));
rddot_des3 = des_state.acc(3) + Kd(3)*(des_state.vel(3)-state.vel(3)) +Kp(3)*(des_state.pos(3)-state.pos(3));
phi_des = (1/params.gravity)*(rddot_des1*sin(des_state.yaw) - rddot_des2*cos(des_state.yaw));
theta_des = (1/params.gravity)*(rddot_des1*cos(des_state.yaw) + rddot_des2*sin(des_state.yaw));
u2=[kpph*(phi_des-state.rot(1))+kdph*(-state.omega(1));kpth*(theta_des-state.rot(2))+kdth*(-state.omega(2));kpps*(des_state.yaw-state.rot(3))+kdps*(des_state.yawdot-state.omega(3))];
u1=params.mass*(params.gravity+rddot_des3);
% Thrust
F = u1;

% Moment
M = u2;

% =================== Your code ends here ===================

end
