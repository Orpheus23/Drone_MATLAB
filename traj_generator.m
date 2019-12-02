function [ desired_state ] = traj_generator(t, state, waypoints)
    % TRAJ_GENERATOR: Generate the trajectory passing through all
    % positions listed in the waypoints list
    %
    % NOTE: This function would be called with variable number of input arguments.
    % During initialization, it will be called with arguments
    % trajectory_generator([], [], waypoints) and later, while testing, it will be
    % called with only t and state as arguments, so your code should be able to
    % handle that. This can be done by checking the number of arguments to the
    % function using the "nargin" variable, check the MATLAB documentation for more
    % information.
    %
    % t,state: time and current state (same variable as "state" in controller)
    % that you may use for computing desired_state
    %
    % waypoints: The 3xP matrix listing all the points you much visited in order
    % along the generated trajectory
    %
    % desired_state: Contains all the information that is passed to the
    % controller for generating inputs for the quadrotor
    %
    % It is suggested to use "persistent" variables to store the waypoints during
    % the initialization call of trajectory_generator.
    
    
    %% Example code:
    % Note that this is an example of naive trajectory generator that simply moves
    % the quadrotor along a stright line between each pair of consecutive waypoints
    % using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
    % should write your own trajectory generator for the submission.
    %{

end
###
%
    %}
    
    %% Fill in your code here
    
    % desired_state.pos = zeros(3,1);
    % desired_state.vel = zeros(3,1);
    % desired_state.acc = zeros(3,1);
    % desired_state.yaw = 0;
    function[t_m] =t_mat1(t)
        t_m=[1 t^1 t^2 t^3 t^4 t^5 t^6 t^7]';
    end
    function[t_m_dot] =t_mat2(t)
        t_m_dot = [0 1 2*t 3*t^2 4*t^3 5*t^4 6*t^5 7*t^6]';
    end
    function[t_m_ddot] =t_mat3(t)
        t_m_ddot = [0 0 2 6*t 12*t^2 20*t^3 30*t^4 42*t^5]';
    end
    function[C]=pols_coef_retrn(n,wayp)
        A = zeros(8*n,8*n);
        B = zeros(1,8*n);
        
        p=1;
        l=0;
        for i=1:n
            k=1;
            for j=1:8
                if j==1
                    
                    A(l+1,l+1)=1;
                    B(1,l+1)=wayp(i);
                    A(l+2,l+1:l+8)=[1 1 1 1 1 1 1 1];
                    B(1,l+2)=wayp(i+1);
                    
                    
                end
                p=p+1;
                if j>=3
                    for theta = l+k+1:l+8
                        
                        if i==n 
                            if k<=3
                                
                                A(l+j,theta)= factorial(theta-l-1)/(factorial(theta-l-1-k));
                                
                            elseif k==4
                                
                                A(l+j,3)=1;
                            
                            elseif k==5
                                
                                A(l+j,4)=1;
                                
                             
                                    
                            elseif k==6
                                
                                A(l+j,2)=1;
                                
                                
                                
                            end
                        
                        else
                            
                            
                            A(l+j,theta)=(factorial(theta-l-1)/(factorial(k)*factorial(theta-l-1-k)));
                            
                            if theta==(l+8)
                                A(l+j,theta+1+k)=-1;
                            end
                        end
                    end
                    k=k+1;
                end
            
            end
        l=l+8;
        end
        
        C=A\(B');
    end
    
    
    
    
    
    
    
    persistent waypoints0 traj_time d0
    if nargin > 2
        d = waypoints(:,2:end) - waypoints(:,1:end-1);
        d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
        traj_time = [0, cumsum(d0)];
        waypoints0 = waypoints;
        
        
    else
        if (t >= traj_time(end))
            t = traj_time(end)-0.0001;
        end
        t_index = find(traj_time>t,1) - 1;
        t_index = max(t_index,1);
        scale = (t-traj_time(t_index)) / d0(t_index);
        if(t == 0)
            desired_state.pos = waypoints0(:,1);
            desired_state.vel = [0;0;0];  %no idea what you want there
            desired_state.acc = [0;0;0];  %no idea what you want there
        else
            
            index = ((t_index-1)*8+1:t_index*8);
            
            nn=size(waypoints0(1,:));
            n=nn(2)-1;
            coeff_x = pols_coef_retrn(n,waypoints0(1,:)');
            coeff_y = pols_coef_retrn(n,waypoints0(2,:)');
            coeff_z = pols_coef_retrn(n,waypoints0(3,:)');
            
            desired_state.pos = [coeff_x(index)'*t_mat1(scale);coeff_y(index)'*t_mat1(scale);coeff_z(index)'*t_mat1(scale);];
            desired_state.vel = [coeff_x(index)'*t_mat2(scale);coeff_y(index)'*t_mat2(scale);coeff_z(index)'*t_mat2(scale);].*(1/d0(t_index));
            desired_state.acc = [coeff_x(index)'*t_mat3(scale);coeff_y(index)'*t_mat3(scale);coeff_z(index)'*t_mat3(scale);].*((1/d0(t_index))^2);
        end
        
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    end
end

