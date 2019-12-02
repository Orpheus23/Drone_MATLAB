function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
%Reorganising the input and output feed
inp_feed=video_pts';
outp_feed=logo_pts';
inp_feed=[inp_feed;1 1 1 1];
outp_feed=[outp_feed;1 1 1 1];
%Use this to get the H value for the input to basis
inp1=inp_feed(:,1:3);
inp2=inp_feed(:,4);
A=inp1.*((inv(inp1)*inp2)');
%Use this to get from basis to output
outp1=outp_feed(:,1:3);
outp2=outp_feed(:,4);
B=outp1.*((inv(outp1)*outp2)');

H = B*(inv(A));

end

