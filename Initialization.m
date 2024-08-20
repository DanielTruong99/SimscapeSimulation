addpath('urdf');
addpath('utility');
addpath('resources/leg/urdf');
addpath('resources/leg/meshes');
addpath('images');
addpath('gen_files');
addpath('controllers');

load('joint_trajectory.mat');

npt = 2; % number of contact points per line
% contact_cloud = [ [[linspace(-0.07,0.13,npt)]',ones(npt,1)*0, ones(npt,1)*(-0.035)]; 
%                     [[linspace(-0.07,0.13,npt)]',ones(npt,1)*0.01, ones(npt,1)*(-0.035)]; 
%                     [[linspace(-0.07,0.13,npt)]',ones(npt,1)*-0.01, ones(npt,1)*(-0.035)]];
contact_cloud = [[[linspace(-0.07,0.13,npt)]',ones(npt,1)*0.17, ones(npt,1)*(-0.035)]; 
                    [[linspace(-0.07,0.13,npt)]',ones(npt,1)*-0.17, ones(npt,1)*(-0.035)]];

motion_time_constant = 1e-3;

LEFT_LEG_FEEDBACK_GAIN.hip = struct('P', 10000, 'I', 500, 'D', 900);
LEFT_LEG_FEEDBACK_GAIN.hip2 = struct('P', 10000, 'I', 500, 'D', 900);
LEFT_LEG_FEEDBACK_GAIN.thight = struct('P', 10000, 'I', 500, 'D', 900);
LEFT_LEG_FEEDBACK_GAIN.calf = struct('P', 10000, 'I', 500, 'D', 900);
LEFT_LEG_FEEDBACK_GAIN.toe = struct('P', 10000, 'I', 500, 'D', 900);

RIGHT_LEG_FEEDBACK_GAIN.hip = struct('P', 10000, 'I', 500, 'D', 900);
RIGHT_LEG_FEEDBACK_GAIN.hip2 = struct('P', 10000, 'I', 500, 'D', 900);
RIGHT_LEG_FEEDBACK_GAIN.thight = struct('P', 10000, 'I', 500, 'D', 900);
RIGHT_LEG_FEEDBACK_GAIN.calf = struct('P', 10000, 'I', 500, 'D', 900);
RIGHT_LEG_FEEDBACK_GAIN.toe = struct('P', 10000, 'I', 500, 'D', 900);

