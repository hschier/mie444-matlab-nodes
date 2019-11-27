 function [rpath, prm] = pathplan(iterations, NumNodes, startLoc, endLoc, mapInflated)
    % Takes in an amout of iterations to preform and
    % the amount of nodes to generate for each PRM instance
    % outputs a path and a prm object in O(inf) time
    bestPath = [];
    bestScore = inf;
    for i = 1:iterations
        rpath = [];
        while size(rpath, 1) == 0
            % define PRM planner
            prm = robotics.PRM(mapInflated);
            prm.NumNodes = NumNodes;
            prm.ConnectionDistance = 3; %max dist between two connected nodes
            rpath = findpath(prm, startLoc, endLoc);
        end
        pathNodeCount = size(rpath,1);
        pathLen = 0;
        for i = 1 : (pathNodeCount - 1)
            pathLen = pathLen + norm(rpath(i), rpath(i+1));
        end
%         score = pathNodeCount * pathLen
        score = pathLen
        if (score < bestScore)
            disp("==================");
            bestScore = score
            bestPath = rpath;
            bestPrm = prm;
        end
    end
    rpath = bestPath;
    prm = bestPrm;
end

