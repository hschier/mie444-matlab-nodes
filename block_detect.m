clear; clc; close all;
rosshutdown;


setenv('ROS_MASTER_URI', 'http://192.168.1.20');
setenv('ROS_HOSTNAME', '192.168.1.20');
rosinit('192.168.1.20','NodeHost','192.168.1.20')

% in degrees
beta_thresh = 10.0;
plot_on = 0;

laserSub = rossubscriber('/scan', 'sensor_msgs/LaserScan');
amclSub = rossubscriber('/amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped');

pointPub = rospublisher('/block_point', 'geometry_msgs/Point');

% clf;
% bag = rosbag('block2.bag');
% bSel = select(bag,'Topic','/scan');
% msgStruct = readMessages(bSel,'DataFormat','struct');
% scanMsg = msgStruct{1};
% bag_counter = 0;
% define inflated map
load('good_map.mat');
map = myOccMap;
inflatedMap = copy(map);
inflate(map, 0.02);
inflate(inflatedMap, 0.08);

block_points = inf (8, 2);
queue_index = 1;

while true
    msg = rosmessage('geometry_msgs/Point');
    msg.X = inf;
    msg.Y = inf;
    msg.Z = inf;
    try
        % Receive laser scan and odometry message.
        scanMsg = receive(laserSub);
        poseMsg = receive(amclSub);
        if (poseMsg.Header.FrameId == "unlocalized_map")
            send(pointPub, msg);

            continue;
        end
        robot_map = [poseMsg.Pose.Pose.Position.X; poseMsg.Pose.Pose.Position.Y; poseMsg.Pose.Pose.Position.Z];

    %     bag_counter = bag_counter + 1;
    %     scanMsg = msgStruct{bag_counter};
    
        scanMsg.Ranges(1:250) = flip(scanMsg.Ranges(1:250));
        scanMsg.Ranges(251:500) = flip(scanMsg.Ranges(251:500));
        ranges = scanMsg.Ranges;
        %ranges = imgaussfilt(ranges,0.01);
        ang_inc = scanMsg.AngleIncrement;
        n = size(scanMsg.Ranges, 1);

        cluster_num = 1;
        clusters = zeros(n, 1);
        pos = zeros(2, n);
        stitch = 0;
        for i=1:n
            curr_index = i;
            prev_index = i-1;
            if i == 1
                prev_index = n;
            end
            if isinf(ranges(i))
                    pos(:,i-1) = pos(:,i-2)+(pos(:,i-2)-pos(:,i-3));
                    ranges(i-1) = sqrt(pos(1,i-1)^2+pos(2,i-1)^2);
                    pos(:,i) = pos(:,i-1)+(pos(:,i-1)-pos(:,i-2));
                    ranges(i) = sqrt(pos(1,i)^2+pos(2,i)^2);
            else
                angle = ang_inc*(prev_index)+scanMsg.AngleMin;
                x = ranges(prev_index)*cos(angle);
                y = ranges(prev_index)*sin(angle);
                pos(:,i) = [x;y];  
            end
            BH = ranges(curr_index)*sin(ang_inc);
            HA = ranges(prev_index)-ranges(curr_index)*cos(ang_inc);
            beta = atan2d(BH,HA);
            clusters(i) = cluster_num;
            if (beta < beta_thresh || sqrt(BH^2 + HA^2)>0.02)
                cluster_num = cluster_num+1;
            end
            if (beta > beta_thresh && i == 1)
                stitch = 1;
            end
        end

        %stitch beginning and end of lidar scan together into one cluster
        val = clusters(n);
        if (stitch == 1)
            for i = 0:n    
                if clusters(n-i) == val
                    clusters(n-i) = 1;
                else
                    val = val - 1;
                    break
                end
            end
        end

        %plot clusters 
        if plot_on
            hold off;
            hold on;
            grid on;
            curr_color = rand(1,3);
            for i = 2:n
                if clusters(i) ~= clusters(i-1)
                    curr_color = rand(1,3);
                end
                if clusters(i) == 1
                    curr_color = [0,0,0];
                end
                scatter3(pos(1,i),pos(2,i),clusters(i), [], curr_color);
            end
        end
            
        %clusters_idx = [first index, last index, eucld distance] of clusters
        clusters_idx = zeros(val, 3);
        for i=1:n
            if i == 1
                clusters_idx(1, 1) = 1;
                continue
            end
            clusters_idx(clusters(i),2) = i;
            %sets end index of previous cluster and start index of new cluster
            %when a new cluster starts
            if clusters(i) ~= clusters(i-1)
                if clusters(i) == 1
                    clusters_idx(1,1) = i;
                else
                    clusters_idx(clusters(i),1) = i;
                end
            end
        end
        
        %find clusters of appropriate euclidean length and not in walls
        for i = 1:val
            x1 = pos(1,clusters_idx(i,1));
            x2 = pos(1,clusters_idx(i,2));
            y1 = pos(2,clusters_idx(i,1));
            y2 = pos(2,clusters_idx(i,2));
            dy = y2-y1;
            dx = x2-x1;
            x_mid = (x1+x2)/2;
            y_mid = (y1+y2)/2;

            clusters_idx(i,3) = sqrt(dx^2+dy^2);
            if(clusters_idx(i,3) > 0.04 && clusters_idx(i,3) < 0.09 && clusters_idx(i,2)-clusters_idx(i,1) > 2)
                if ranges(mod(clusters_idx(mod(i-2,val)+1,2)-2,n)+1)-0.03 < ranges(mod(clusters_idx(i,1)-2,n)+1) || ranges(mod(clusters_idx(mod(i,val)+1,1)-2,n)+1)-0.03 < ranges(mod(clusters_idx(i,2)-2,n)+1)
                    continue
                else
                    block_cluster = i;
                    block_origin = [x_mid ; y_mid];
                    block_origin = block_origin + [sign(x_mid)*abs(0.025*cos(pi/2-atan(dy/dx))); sign(y_mid)*abs(0.025*sin(pi/2-atan(dy/dx)))];             
                    if plot_on
                        scatter3(block_origin(1),block_origin(2),i, [], [0,0,0]);
                    end
                    R = [cos(robot_map(3)) -sin(robot_map(3)); sin(robot_map(3)) cos(robot_map(3))];
                    block_map = R*block_origin + robot_map(1:2);

                    %block and robot desired pose in  map frame
                    %robot_map = [rob_x rob_y rob_theta] from localized pose
                    %block_map = block_origin + robot_map(1:2);
%                     msg.X = block_map(1);
%                     msg.Y = block_map(2);
%                     msg.Z = 0;
%                     if (checkOccupancy(map, block_map') == 0)
%                         scatter3(block_origin(1),block_origin(2),i, [], [0,0,0]);
                    block_points(queue_index,:) = block_map;
                    queue_index = mod(queue_index,20) + 1;
%                         break
%                     end
                end
            end
        end
        
        %scatter3(closest(1)-robot_map(1), closest(2)-robot_map(2), 99, [], [0,0,0]);
        block_mean = mean(block_points);
        normie = 0;
        for k = 1:size(block_points, 1)
            normie = normie + norm (block_points(k,:)- block_mean);
        end
        normie
        if normie < 1.5
            msg.X = block_mean(1);
            msg.Y = block_mean(2);
            msg.Z = 0;
            %poses in a radius around the block
            radius = 0.125;
            robot_poses = zeros(3,size(block_points, 1));
            robot_poses(3,:) = linspace(0,2*pi,size(block_points, 1));
            robot_poses(1:2,:) = block_mean + [radius*cos(pi+robot_poses(3,:)) ; radius*sin(pi+robot_poses(3,:))];

            %finds points that are possible for the robot to move to
            for j = 1:size(block_points, 1)
                if (checkOccupancy(inflatedMap, robot_poses(1:2,j)')~=0)
                    robot_poses(:,j) = inf;
                end
            end

            dist = sum((robot_poses(1:2,:)'-robot_map(1:2)) .^2,2);
            closest = robot_poses(:,dist == min(dist));
        end 
    catch e
        disp(e.message)
    end
    send(pointPub, msg);
end

    


