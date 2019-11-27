clear; clc; close all;
rosshutdown;

setenv('ROS_MASTER_URI', 'http://192.168.1.20');
setenv('ROS_HOSTNAME', '192.168.1.20');
rosinit('192.168.1.20','NodeHost','192.168.1.20')

clear
close all;

%%
% lost, waypoint, to_block, drop_block
state = "lost";

% load map
load good_map.mat
load centroids.mat
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

receive(amclSub);

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
        disp("Stopping robot");
        stopMsg = rosmessage('geometry_msgs/Twist');
        stopMsg.Linear.X = 0;
        stopMsg.Angular.Z = 0;
        send(cmdVelPub, stopMsg);
        
        state = "waypoint";
        
        endLocation = [4.1437  2.9509];
        
        [robotPath, prm] = pathplan(40, 40, robotCurrentPose(1:2)', endLocation, mapInflated);
        
%         for i = 1:size(robotPath,1)
%             disp("New way point waypoint at:");
%             [d, ix] = min (norm(centerMap - robotPath(i,:)));
%             robotPath(i,:) = centerMap (ix,:);
%             disp(centerMap (ix,:));
%         end
        
        disp("found a path, showing PRM...");
        show(prm);
        xlim([1.75 4.5]);
        ylim([1.7 3.2]);
        
        robotGoal = robotPath(end,:);
        robotInitialLocation = robotPath(1,:);
        disp("robot goal is:");
        disp(robotGoal);
        disp("initial location is");
        disp(robotInitialLocation);
    elseif amclMsg.Header.FrameId == "unlocalized_map"
       state = "lost";
    end
    
%     if state == "waypoint" && ~isinf(blockPoseMsg.X)
%         state = "get_block";
%         robotPath = [blockPoseMsg.X blockPoseMsg.Y];
%         
%     end
     
    if state == "lost"
        disp("state is lost")
        avoidMsg = receive(avoidSub);
        send(cmdVelPub, avoidMsg);
       
    elseif state == "waypoint"
        disp("state is waypoint")
        if ( size(robotPath, 1) == 0 ) % reached final waypoint
            msg.Linear.X = 0;
            msg.Angular.Z = 0;
        end
        
        angleDiff = atan2(robotPath(1,2)-robotCurrentPose(2), ...
                       robotPath(1,1)-robotCurrentPose(1)) - robotCurrentPose(3);
        distToNext = norm(robotPath(1,:)' - robotCurrentPose(1:2));
        msg = rosmessage('geometry_msgs/Twist');
        
        avoidMsg = receive(avoidSub);

        if (distToNext > 0.04) % not at current waypoint     
%             if (avoidMsg.Linear.X > 0 && abs(angleDiff) < deg2rad(5))
             if can_move_foward(laserMsg) && abs(angleDiff) < deg2rad(15)
                disp("Moving Forward, dist to next is: ");
                disp(distToNext);
                % go forwards!
                msg.Linear.X = min(distToNext*1.2, 0.07);
                disp("moving at rate");
                disp(msg.Linear.X);
            else
                disp("Adjusting Angle, diff is: ");
                disp(angleDiff);
                msg.Linear.X = 0;
                % turn!
                if (angleDiff > 0)
                    msg.Angular.Z = min(angleDiff*0.2, 0.5);
                else
                    msg.Angular.Z = max(angleDiff*0.2, -0.5);
                end
                disp("Turning at rate:");
                disp(msg.Angular.Z);
            end
        else % not at end but have reach a waypoint
            % pop off the current waypoint
            robotPath(1, :) = [];
            disp("Popping waypoint");
        end
        
        [distToNext angleDiff msg.Linear.X msg.Angular.Z]
        send(cmdVelPub, msg);
        
%     elseif state == "get_block"
%         disp("state is get_block");
%         distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
%         angleDiff = atan2(robotPath(1,2)-robotCurrentPose(2), ...
%                        robotPath(1,1)-robotCurrentPose(1)) - robotCurrentPose(3);
%         distToNext = norm(robotPath(1,:)' - robotCurrentPose(1:2));
%         msg = rosmessage('geometry_msgs/Twist');
%         
%         avoidMsg = receive(avoidSub);
% 
%         if (distToNext > 0.04) % not at current waypoint     
%             if (avoidMsg.Linear.X > 0 && abs(angleDiff) < deg2rad(5))
%                  disp("Moving Forward, dist to next is: ");
%                 distToNext
%                 % go forwards!
%                 msg.Linear.X = min(distToNext*1.2, 0.07);
%             else
%                 disp("Adjusting Angle, diff is: ");
%                 angleDiff
%                 msg.Linear.X = 0;
%                 % turn!
%                 if (angleDiff > 0)
%                     msg.Angular.Z = min(angleDiff*0.2, 0.5);
%                 else
%                     msg.Angular.Z = max(angleDiff*0.2, -0.5);
%                 end   
%             end
%         else % not at end but have reach a waypoint
%             % pop off the current waypoint
%             robotPath(1, :) = [];
%             disp("Popping waypoint");
%         end
%         [distToNext angleDiff msg.Linear.X msg.Angular.Z]
        
    end
end
