function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

   %% Place parameters like covarainces, etc. here:
    dt = 0.33;
       
	  % Noise covariance matrices
    Sigma_m = [0.01, 0, 0, 0; 0, 0.001, 0, 0; 0, 0, 0.001, 0; 0, 0, 0, 0.001];
    Sigma_o = [0.001, 0;0, 0.001];
	
	  % Transition matrix
    A = [1 0 dt 0;0 1 0 dt;0 0 1 0;0 0 0 1]; 
    
    % Observation matrix
    H = [1 0 0 0;0 1 0 0];
    
    % Check if the first time running this function
    if previous_t<0           
        state = [x, y, 0, 0]';
        param.P = 0.1*eye(4,4);
        predictx = x;
        predicty = y;
        return;
    end
    
    % Define the previous state
    Ptmin1=param.P;
    %% TODO: Add Kalman filter updates  
    % Predicted state & covariance matrix
    Xtp = A*state;
    Ptp = A*Ptmin1*(A')+Sigma_m;
    R= Sigma_o;
    % Kalman gain
    K = (Ptp*(H'))*inv(H*Ptp*(H')+R);
    % Y measrement
    Y = H*[x y 0 0]';
      
    % Update covariance matrix  
    param.P = (eye(4,4)-K*H)*Ptp;  
    
    % Update state
    state = Xtp+K*(Y-H*Xtp);
    % Prediction
    predictx = state(1);
    predicty = state(2);
    
end
