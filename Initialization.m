addpath('urdf');
addpath('utility');
addpath('resources/leg/urdf');
addpath('resources/leg/meshes');
addpath('images');
addpath('gen_files');
addpath('controllers');

load('joint_trajectory_filtered_2.mat');

npt = 3; % number of contact points per line
contact_cloud = [ [[linspace(-0.07,0.13,npt)]',ones(npt,1)*0, ones(npt,1)*(-0.035)]; 
                    [[linspace(-0.07,0.13,npt)]',ones(npt,1)*0.01, ones(npt,1)*(-0.035)]; 
                    [[linspace(-0.07,0.13,npt)]',ones(npt,1)*-0.01, ones(npt,1)*(-0.035)]];
% contact_cloud = [[[linspace(-0.07,0.13,npt)]',ones(npt,1)*0.17, ones(npt,1)*(-0.035)]; 
%                     [[linspace(-0.07,0.13,npt)]',ones(npt,1)*-0.17, ones(npt,1)*(-0.035)]];

motion_time_constant = 1e-3;

LEFT_LEG_FEEDBACK_GAIN.hip = struct('P', 150, 'I', 0, 'D', 5);
LEFT_LEG_FEEDBACK_GAIN.hip2 = struct('P', 150, 'I', 0, 'D', 5);
LEFT_LEG_FEEDBACK_GAIN.thight = struct('P', 100, 'I', 0, 'D', 5);
LEFT_LEG_FEEDBACK_GAIN.calf = struct('P', 200, 'I', 0, 'D', 5);
LEFT_LEG_FEEDBACK_GAIN.toe = struct('P', 20, 'I', 0, 'D', 2);

RIGHT_LEG_FEEDBACK_GAIN.hip = struct('P', 150, 'I', 0, 'D', 5);
RIGHT_LEG_FEEDBACK_GAIN.hip2 = struct('P', 150, 'I', 0, 'D', 5);
RIGHT_LEG_FEEDBACK_GAIN.thight = struct('P', 200, 'I', 0, 'D', 5);
RIGHT_LEG_FEEDBACK_GAIN.calf = struct('P', 200, 'I', 0, 'D', 5);
RIGHT_LEG_FEEDBACK_GAIN.toe = struct('P', 20, 'I', 0, 'D', 2);

