clear; clc; close all;
rosshutdown;

setenv('ROS_MASTER_URI', 'http://192.168.1.20');
setenv('ROS_HOSTNAME', '192.168.1.20');
rosinit('192.168.1.20','NodeHost','192.168.1.20')

clear
close all;

%
% lost, waypoint, to_block, drop_block
state = "lost";

% load map
load good_map.mat
load search_path.mat
loadingZones =   [  3.5624    2.7731;
                    2.6962    2.0630;
                    2.1031    2.0084;
                    2.0329    2.8122];
endZone = loadingZones(4, :);


occMat = occupancyMatrix(myOccMap);
occMat = flip(occMat, 1);
res = myOccMap.Resolution;
myOccMap = occupancyMap(occMat, res);

% define robot
robot = differentialDriveKinematics("WheelRadius", 0.034, ...
         "WheelSpeedRange", [-0.1, -0.1], "TrackWidth", 0.2,  ...
         "VehicleInputs", "VehicleSpeedHeadingRate");

% define inflated map
% mapInflated = copy(myOccMap);
% inflate(mapInflated, robot.TrackWidth/2 + 0.025);

inflatedMat = imread('inflatedboi3.pgm'); % he beeg
mapInflated = occupancyMap(double(inflatedMat)/255, res);


%
avoidSub = rossubscriber('/avoid_vel', 'geometry_msgs/Twist');
cmdSpecialPub = rospublisher('/cmd_special', 'std_msgs/String');

laserSub = rossubscriber('/scan', 'sensor_msgs/LaserScan');
amclSub = rossubscriber('/amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped');
cmdVelPub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
blockPoseSub = rossubscriber('/block_point', 'geometry_msgs/Point');

receive(amclSub);

while true 
    % get lasteest odom
    amclMsg = amclSub.LatestMessage;
    laserMsg = receive(laserSub);
    blockPoseMsg = blockPoseSub.LatestMessage;
%     blockLocation = [blockPoseMsg.X blockPoseMsg.Y];

    robotCurrentPose =  [amclMsg.Pose.Pose.Position.X; amclMsg.Pose.Pose.Position.Y; amclMsg.Pose.Pose.Position.Z];

    % check if we are not lost and are localized
    if state == "lost" && amclMsg.Header.FrameId == "localized_map" && ...
            checkOccupancy(mapInflated, robotCurrentPose(1:2)') == 0
        %%
        disp("To waypoint state");

        % STOP
        disp("Stopping robot");
        stopMsg = rosmessage('geometry_msgs/Twist');%         send(cmdVelPub, stopMsg);

        stopMsg.Linear.X = 0;
        stopMsg.Angular.Z = 0.0;
        send(cmdVelPub, stopMsg);
        
        state = "waypoint";
%         endLocation = [4.1437  2.9509];   
        endLocation = searchPath(1, :);
        
        tic
        [robotPath, prm] = pathplan(5, 200, robotCurrentPose(1:2)', endLocation, mapInflated);
        toc
        
        disp("found a path, showing PRM...");
        show(prm);
        xlim([1.75 4.5]);
        ylim([1.7 3.2]);
        
        %append searchPath
        robotPath = [robotPath; searchPath(2:end, :)]
        
        robotInitialLocation = robotPath(1,:);
        disp("initial location is");
        disp(robotInitialLocation);
        
        pause(6);
    elseif amclMsg.Header.FrameId == "unlocalized_map"
       state = "lost";
       
    elseif state == "waypoint" && ~isinf(blockPoseMsg.X)
        state = "to_block";
        disp("============================");
        disp("TOOOO BLOOOCCKKK");
        disp("============================");
        
        stopMsg.Linear.X = 0;
        stopMsg.Angular.Z = 0.0;
        send(cmdVelPub, stopMsg);
        pause(5)
        blockPoseMsg = receive(blockPoseSub);
        blockLocation = [blockPoseMsg.X blockPoseMsg.Y];
        if (isinf(blockPoseMsg.X))
           state = "waypoint"; 
        end
        
    elseif state == "lost_home" && checkOccupancy(mapInflated, robotCurrentPose(1:2)') == 0
        stopMsg.Linear.X = 0;
        stopMsg.Angular.Z = 0.0;
        send(cmdVelPub, stopMsg);
        state = "go_home";
        tic
        [robotPath, prm] = pathplan(5, 200, robotCurrentPose(1:2)', endZone, mapInflated);
        toc
        disp("found a path to home, showing PRM...");
        show(prm);  
    end
    
    if state == "lost" || state == "lost_home"
        %% if we are lost
        disp("state is lost")
        state
        avoidMsg = receive(avoidSub);
        send(cmdVelPub, avoidMsg);
        
    elseif state == "waypoint" || state == "go_home" 
        %% if we are going to a waypoint
        disp("state is waypoint")
        if ( size(robotPath, 1) == 0) % reached final waypoint
            msg.Linear.X = 0;
            msg.Angular.Z = 0;
            if (state == "go_home")
                disp("I'm home!!!!!!!!!!!")
                msgGripper = rosmessage('std_msgs/String');
                msgGripper.Data = "open";
                send(cmdSpecialPub, msgGripper);
            end
            
            break;
        end
        
        angleDiff = atan2(robotPath(1,2)-robotCurrentPose(2), ...
                       robotPath(1,1)-robotCurrentPose(1)) - robotCurrentPose(3);
        if (angleDiff > pi)
            angleDiff = 2*pi - angleDiff;
        elseif (angleDiff < -pi)
            angleDiff = 2*pi + angleDiff;
        end
        distToNext = norm(robotPath(1,:)' - robotCurrentPose(1:2));
        msg = rosmessage('geometry_msgs/Twist');
        
        
        if (distToNext > 0.06) % not at current waypoint   
            disp("Not at the current waypoint")
            [left, center, right] = sample_controls(laserMsg);
            
            % If we can move forward and we are at the right angle
            if center && abs(angleDiff) < deg2rad(15)
                disp("Moving Forward, dist to next is: ");
                disp(distToNext);
                % go forwards!
                msg.Linear.X = min(distToNext*1.2, 0.045);
                
                if (angleDiff > 0)
                    msg.Angular.Z = min(angleDiff*0.2, 0.5);
                else
                    msg.Angular.Z = max(angleDiff*0.2, -0.5);
                end
            elseif abs(angleDiff) < deg2rad(15) && left
                disp("Swerve left to avoid wall on the right: ");
                msg.Linear.X = 0.01;
                msg.Angular.Z = min(angleDiff*0.2, 0.5);
            elseif abs(angleDiff) < deg2rad(15) && right
                disp("Swerve right to avoid wall on the left: ");
                msg.Linear.X = 0.01;
                msg.Angular.Z = max(angleDiff*0.2, -0.5);
            elseif (abs(angleDiff) < deg2rad(3))
                disp("I can't do anything, initializing random drive :(")
                count = 0;
                while (count < 2)
                    send(cmdVelPub, receive(avoidSub));
                    count = count + 1;
                end
             else
                disp("Our angle is too far off, adjusting...");
                disp(angleDiff);
                msg.Linear.X = 0;
                % turn!
                if (angleDiff > 0)
                    msg.Angular.Z = min(angleDiff*0.2, 0.5);
                else
                    msg.Angular.Z = max(angleDiff*0.2, -0.5);
                end
            end
            disp("moving at rate");
            disp(msg.Linear.X);
            disp("Turning at rate:");
            disp(msg.Angular.Z);
        else % not at end but have reach a waypoint
            % pop off the current waypoint
            robotPath(1, :) = [];
            disp("===============================================");
            disp("Popping waypoint");
            disp("===============================================");
        end
        
        [distToNext angleDiff msg.Linear.X msg.Angular.Z];
        send(cmdVelPub, msg);
    elseif state == "to_block"
        disp("I'm in to_block state");
        
        %do planning in local frame
        angleDiff = atan2(blockLocation(2)-robotCurrentPose(2), blockLocation(1)-robotCurrentPose(1)) - robotCurrentPose(3) ;
        if (angleDiff > pi)
            angleDiff = 2*pi - angleDiff;
        elseif (angleDiff < -pi)
            angleDiff = 2*pi + angleDiff;
        end
        distToNext = norm(blockLocation' - robotCurrentPose(1:2))
        if (distToNext > 0.075) % not at current waypoint               
            % If we can move forward and we are at the right angle
            if abs(angleDiff) < deg2rad(5)
                disp("Moving Forward, dist to next is: ");
                disp(distToNext);
                % go forwards!
                msg.Linear.X = min(distToNext*1.2, 0.045);
                
                if (angleDiff > 0)
                    msg.Angular.Z = min(angleDiff*0.2, 0.5);
                else
                    msg.Angular.Z = max(angleDiff*0.2, -0.5);
                end
             else
                disp("Our angle is too far off, adjusting...");
                disp(angleDiff);
                msg.Linear.X = 0;
                % turn!
                if (angleDiff > 0)
                    msg.Angular.Z = min(angleDiff*0.2, 0.5);
                else
                    msg.Angular.Z = max(angleDiff*0.2, -0.5);
                end
            end
            disp("moving at rate");
            disp(msg.Linear.X);
            disp("Turning at rate:");
            disp(msg.Angular.Z);
            send(cmdVelPub, msg);

        else % reached end of block, smack
            msgGripper = rosmessage('std_msgs/String');
            msgGripper.Data = "close";
            disp("closing the gripper!!!!!!!!!!!!!");
            pause(3)
            send(cmdSpecialPub, msgGripper);
%             state = "go_home";
            disp("Going home");
            state = "lost_home";
        end
        msgGripper = rosmessage('std_msgs/String');
        msgGripper.Data = "red";
        send(cmdSpecialPub, msgGripper);
    end
    
  
    if state == "go_home"
        if ( size(robotPath, 1) == 0 ) % reached final waypoint
            msgGripper = rosmessage('std_msgs/String');
            msgGripper.Data = "open";
            send(cmdSpecialPub, msgGripper);
        end
    end
end
