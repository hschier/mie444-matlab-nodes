setenv('ROS_MASTER_URI', 'http://192.168.1.20');
setenv('ROS_HOSTNAME', '192.168.1.20');
rosinit('192.168.1.20','NodeHost','192.168.1.20')
%%
% lost, waypoint, to_block, drop_block
state = "lost";

% load map
load good_map.mat
occMat = occupancyMatrix(myOccMap);
occMat = flip(occMat, 1);
res = myOccMap.Resolution;
myOccMap = occupancyMap(occMat, res);

% define robot
robot = differentialDriveKinematics("WheelRadius", 0.034, ...
         "WheelSpeedRange", [-0.1, -0.1], "TrackWidth", 0.2,  ...
         "VehicleInputs", "VehicleSpeedHeadingRate");

% define inflated map
mapInflated = copy(myOccMap);
inflate(mapInflated, robot.TrackWidth/2 + 0.025);


%%
avoidSub = rossubscriber('/avoid_vel', 'geometry_msgs/Twist');

laserSub = rossubscriber('/scan', 'sensor_msgs/LaserScan');
amclSub = rossubscriber('/amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped');
cmdVelPub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
blockPoseSub = rossubscriber('/block_pose', 'geometry_msgs/Point');

while true
    % get lasteest odom
    amclMsg = amclSub.LatestMessage;
    laserMsg = laserSub.LatestMessage;
    blockPoseMsg = blockPoseSub.LatestMessage;
    robotCurrentPose =  [amclMsg.Pose.Pose.Position.X; amclMsg.Pose.Pose.Position.Y; amclMsg.Pose.Pose.Position.Z];
                
    if state == "lost" && amclMsg.Header.FrameId == "localized_map" && ...
            checkOccupancy(mapInflated, robotCurrentPose(1:2)') == 0
        disp("To waypoint state");

        % STOP
        stopMsg = rosmessage('geometry_msgs/Twist');
        stopMsg.Linear.X = 0;
        stopMsg.Angular.Z = 0;
        send(cmdVelPub, stopMsg);
        
        state = "waypoint";
        
        endLocation = [4.1437  2.9509];
        
        [robotPath, prm] = pathplan(40, 40, robotCurrentPose(1:2)', endLocation, mapInflated);
        
        for i = 1:size(robotPath,1)
            [d, ix] = min (norm(center_map - robotPath(i,:));
            robotPath(i,:) = center_map (ix,:);
        end
        
        disp("found a path");
        show(prm);
        xlim([1.75 4.5]);
        ylim([1.7 3.2]);
        
        robotGoal = robotPath(end,:);
        robotInitialLocation = robotPath(1,:);
    end
    
    if state == "waypoint" && ~isinf(blockPoseMsg.X)
        state = "get_block";
        robotPath = [blockPoseMsg.X blockPoseMsg.Y];
        
    end
     
    if state == "lost"
        avoidMsg = receive(avoidSub);
        send(cmdVelPub, avoidMsg);
       
    elseif state == "waypoint"
        if ( size(robotPath, 1) == 0 ) % reached final waypoint
            msg.Linear.X = 0;
            msg.Angular.Z = 0;
%             state = "grab_block";
        end
        
%         distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
        angleDiff = atan2(robotPath(1,2)-robotCurrentPose(2), ...
                       robotPath(1,1)-robotCurrentPose(1)) - robotCurrentPose(3);
        distToNext = norm(robotPath(1,:)' - robotCurrentPose(1:2));
        msg = rosmessage('geometry_msgs/Twist');
        
        avoidMsg = receive(avoidSub);

        if (distToNext > 0.04) % not at current waypoint     
            if (avoidMsg.Linear.X > 0 && abs(angleDiff) < deg2rad(5))
                 disp("Moving Forward, dist to next is: ");
                distToNext
                % go forwards!
                msg.Linear.X = min(distToNext*1.2, 0.07);
            else
                disp("Adjusting Angle, diff is: ");
                angleDiff
                msg.Linear.X = 0;
                % turn!
                if (angleDiff > 0)
                    msg.Angular.Z = min(angleDiff*0.2, 0.5);
                else
                    msg.Angular.Z = max(angleDiff*0.2, -0.5);
                end   
            end
        else % not at end but have reach a waypoint
            % pop off the current waypoint
            robotPath(1, :) = [];
            disp("Popping waypoint");
        end
        [distToNext angleDiff msg.Linear.X msg.Angular.Z]
%         send(cmdVelPub, msg);
    elseif state == "get_block"
        distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
        angleDiff = atan2(robotPath(1,2)-robotCurrentPose(2), ...
                       robotPath(1,1)-robotCurrentPose(1)) - robotCurrentPose(3);
        distToNext = norm(robotPath(1,:)' - robotCurrentPose(1:2));
        msg = rosmessage('geometry_msgs/Twist');
        
        avoidMsg = receive(avoidSub);

        if (distToNext > 0.04) % not at current waypoint     
            if (avoidMsg.Linear.X > 0 && abs(angleDiff) < deg2rad(5))
                 disp("Moving Forward, dist to next is: ");
                distToNext
                % go forwards!
                msg.Linear.X = min(distToNext*1.2, 0.07);
            else
                disp("Adjusting Angle, diff is: ");
                angleDiff
                msg.Linear.X = 0;
                % turn!
                if (angleDiff > 0)
                    msg.Angular.Z = min(angleDiff*0.2, 0.5);
                else
                    msg.Angular.Z = max(angleDiff*0.2, -0.5);
                end   
            end
        else % not at end but have reach a waypoint
            % pop off the current waypoint
            robotPath(1, :) = [];
            disp("Popping waypoint");
        end
        [distToNext angleDiff msg.Linear.X msg.Angular.Z]
        
    end
end

% plan path
% startLocation = [0.0 0.0];
% endLocation = [0.0 0.0];
% path = findpath(prm, startLocation, endLocation);
% 
% show(prm);
% 
% release(controller);
% controller.Waypoints = path;
% robotGoal = path(end,:);
% robotInitialLocation = path(1,:);
% initialOrientation = 0;
% robotCurrentPose = [robotInitialLocation initialOrientation]';
% distanceToGoal = norm(robotInitialLocation - robotGoal);
% goalRadius = 0.1;
% 

% 
% % define PRM planner
% prm = robotics.PRM(mapInflated);
% prm.NumNodes = 150;
% prm.ConnectionDistance = 0.5; %max dist between two connected nodes
% 
% % define controller params
% controller = controllerPurePursuit;
% controller.DesiredLinearVelocity = 0.1;
% controller.MaxAngularVelocity = 0.9;
% controller.LookaheadDistance = 0.3;
% 
% pause(5);
% amclSub = rossubscriber('/amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped');
% amclMsg = receive(amclSub);
% cmdVelPub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
% 
% 
% % plan path
% startLocation = [0.0 0.0];
% endLocation = [0.0 0.0];
% path = findpath(prm, startLocation, endLocation);
% 
% show(prm);
% 
% release(controller);
% controller.Waypoints = path;
% robotGoal = path(end,:);
% robotInitialLocation = path(1,:);
% initialOrientation = 0;
% robotCurrentPose = [robotInitialLocation initialOrientation]';
% distanceToGoal = norm(robotInitialLocation - robotGoal);
% goalRadius = 0.1;
% 
% 
% % Initialize the simulation loop
% sampleTime = 0.1;
% vizRate = rateControl(1/sampleTime);
% 
% reset(vizRate);
% 
% % Initialize the figure
% figure
% 
% while( distanceToGoal > goalRadius )
%     
%     % Compute the controller outputs, i.e., the inputs to the robot
%     [v, omega] = controller(robotCurrentPose);
%     
%     msg = rosmessage('geometry_msgs/Twist');
%     msg.Linear.X = v;
%     msg.Angular.Z = omega;
%     send(cmdVelPub, msg);
%     
%     % Get the robot's velocity using controller inputs
%     vel = derivative(robot, robotCurrentPose, [v omega]);
%     
%     % Update the current pose
%     robotCurrentPose = robotCurrentPose + vel*sampleTime; 
%     
%     % Re-compute the distance to the goal
%     distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
%     
%     % Update the plot
%     hold off
%     show(map);
%     hold all
% 
%     % Plot path each instance so that it stays persistent while robot mesh
%     % moves
%     plot(path(:,1), path(:,2),"k--d")
%     
%     % Plot the path of the robot as a set of transforms
%     plotTrVec = [robotCurrentPose(1:2); 0];
%     plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
%     plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);
%     light;
%     xlim([0 27])
%     ylim([0 26])
%     
%     waitfor(vizRate);
% end