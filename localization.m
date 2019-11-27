clear; clc; close all;
rosshutdown;

setenv('ROS_MASTER_URI', 'http://192.168.1.20');
setenv('ROS_HOSTNAME', '192.168.1.20');
rosinit('192.168.1.20','NodeHost','192.168.1.20')

%% load map
load good_map.mat
occMat = occupancyMatrix(myOccMap);
occMat = flip(occMat, 1);
res = myOccMap.Resolution;
myOccMap = occupancyMap(occMat, res);
myLIDARmap = myOccMap;

%%  odom model
odometryModel = odometryMotionModel;

% Rotational error due to rotational motion
% Rotational error due to translational motion
% Translational error due to translation motion
% Translational error due to rotational motion
odometryModel.Noise = [1.5 300 40 0.5];

%% laser model
rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.12 2];
rangeFinderModel.Map = myLIDARmap;
rangeFinderModel.NumBeams = 500;
rangeFinderModel.MaxLikelihoodDistance = 0.02;
rangeFinderModel.MeasurementNoise = 0.03;
rangeFinderModel.ExpectedMeasurementWeight = 1;
rangeFinderModel.RandomMeasurementWeight = 0;

%% tf from laser to base
rangeFinderModel.SensorPose = [0 0 0];

laserSub = rossubscriber('/scan', 'sensor_msgs/LaserScan');
odomSub = rossubscriber('/pose', 'geometry_msgs/Pose');

amclPub = rospublisher('/amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped');


%% AMCL obj
amcl = monteCarloLocalization;
amcl.UseLidarScan = true;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;
amcl.UpdateThresholds = [0.0, 0.0, deg2rad(3)];
amcl.ResamplingInterval = 1;
amcl.ParticleLimits = [5000 10000];
amcl.GlobalLocalization = true;
% amcl.InitialPose = [2.0638, 2.8437, 0.0];
% amcl.InitialCovariance = diag([0.5, 0.5, 1]);

visualizationHelper = ExampleHelperAMCLVisualization(myLIDARmap);

xlim([1.75 4.5]);
ylim([1.7 3.2]);

num_cycles = 0;

localized_once = false;

while true
    % Receive laser scan and odometry message.
    scanMsg = receive(laserSub);
    odompose = odomSub.LatestMessage;
    
    n = size(scanMsg.Ranges, 1);
    for i = 1:n
        if scanMsg.Intensities(i) < 253
            scanMsg.Ranges(i) = inf;
        end
    end
    
    scanMsg.Ranges(1:250) = flip(scanMsg.Ranges(1:250));
    scanMsg.Ranges(251:500) = flip(scanMsg.Ranges(251:500));
    scanMsg.Ranges = circshift(scanMsg.Ranges, -6); % account for mechanical error in laser angle

    % Create lidarScan object to pass to the AMCL object.
    scan = lidarScan(scanMsg);
     
    % For sensors that are mounted upside down, you need to reverse the
    % order of scan angle readings using 'flip' function.
    
    % Compute robot's pose [x,y,yaw] from odometry message.
    odomQuat = [odompose.Orientation.W, odompose.Orientation.X, odompose.Orientation.Y, odompose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Position.X, odompose.Position.Y odomRotation(1)];

    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [isUpdated, estimatedPose, estimatedCovariance] = amcl(pose, scan);
    
    normie = norm(estimatedCovariance,'fro')
    
    msg = rosmessage('geometry_msgs/PoseWithCovarianceStamped');
%     msg.Header.FrameId = 'unlocalized_map';
    if (normie > 0.07)
        msg.Header.FrameId = 'unlocalized_map';
    elseif (normie > 0.1 && localized_once)
        msg.Header.FrameId = 'unlocalized_map';
    else
        msg.Header.FrameId = 'localized_map';
        localized_once = true;
    end
    
    msg.Pose.Pose.Position.X = estimatedPose(1);
    msg.Pose.Pose.Position.Y = estimatedPose(2);
    msg.Pose.Pose.Position.Z = estimatedPose(3);
    send(amclPub, msg);
        
    % Drive robot to next pose.
    % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        disp("successful amcl update");
        num_cycles = num_cycles + 1;
        plotStep(visualizationHelper, amcl, estimatedPose, scan, num_cycles)
    end
%     showNoiseDistribution(odometryModel);
end
% 
% stop(wanderHelper);
% 
% 

