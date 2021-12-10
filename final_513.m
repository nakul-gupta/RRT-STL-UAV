%% Set Up
%clc
%close all;

%% Set RNG seed for repeatable result
%rng(1,"twister");

mapData = load("uavMapCityBlock.mat","omap");
omap = mapData.omap;
% Consider unknown spaces to be unoccupied
omap.FreeThreshold = omap.OccupiedThreshold;
%With or without inflate?
%inflate(omap,1);

startPose = [194 107 20 0 0 0 1];
dropOffPose = [94 156 52 0 0 0 1];
goalPose = [76 40 70 0 0 0 1];

figure("Name","StartAndGoal")
hMap = show(omap);
hold on
scatter3(hMap,startPose(1),startPose(2),startPose(3),30,"red","filled")
scatter3(hMap,dropOffPose(1),dropOffPose(2),dropOffPose(3),30,"blue","filled")
scatter3(hMap,goalPose(1),goalPose(2),goalPose(3),30,"green","filled")
hold off
view([-31 63])

%% custom state space -> we have to see about validator map
% custom STL-RRT compared agaisnt the out of the box RRT
% interpolation/connection of waypoints

qrss = stateSpaceSE3([0 200; 0 200; 100 200; ...
    Inf Inf; ...
    Inf Inf; ...
    Inf Inf; ...
    Inf Inf]);

sv = validatorOccupancyMap3D(qrss,"Map",omap);
%sv.ValidationDistance = 0.1;
sv.ValidationDistance = 5;

%% Parameters
threshold = .5;
upper_z = 120;
lower_z = 8;
lateral_bound = 8;
num_neighbors = 20;

%% RRT
% [pthObj, solnInfo] = rrt_stl(qrss, sv, startPose, goalPose, 4000, 20, omap, ...
%    threshold, upper_z, lower_z, lateral_bound, num_neighbors);

%% Error checking

%stateCheck = isStateValid(sv, pthObj.States)


% for i = 1:size(pthObj.States,1)-1
%     %disp(pthObj.States(i,:));
%     [isValid, lastValid] = isMotionValid(sv, pthObj.States(i,:), pthObj.States(i+1,:));
%     if isValid == 0
%         disp("error");
%     end
% end

%% Final Robustness
% robustness = robustnessCalculator(pthObj.States, omap, lateral_bound, ...
%     upper_z, lower_z);
robustness_matrix = zeros(50,7);
for c = 1:50
    [pthObj, solnInfo] = rrt_stl(qrss, sv, startPose, goalPose, 4000, 20, omap, ...
        0, upper_z, lower_z, lateral_bound, 20);
    robustness_matrix(c,1) = robustnessCalculator(pthObj.States, omap, lateral_bound, ...
        upper_z, lower_z);
    [pthObj, solnInfo] = rrt_stl(qrss, sv, startPose, goalPose, 4000, 20, omap, ...
        .25, upper_z, lower_z, lateral_bound, 20);
    robustness_matrix(c,2) = robustnessCalculator(pthObj.States, omap, lateral_bound, ...
        upper_z, lower_z);
    [pthObj, solnInfo] = rrt_stl(qrss, sv, startPose, goalPose, 4000, 20, omap, ...
        .5, upper_z, lower_z, lateral_bound, 20);
    robustness_matrix(c,3) = robustnessCalculator(pthObj.States, omap, lateral_bound, ...
        upper_z, lower_z);
    [pthObj, solnInfo] = rrt_stl(qrss, sv, startPose, goalPose, 4000, 20, omap, ...
        .75, upper_z, lower_z, lateral_bound, 20);
    robustness_matrix(c,3) = robustnessCalculator(pthObj.States, omap, lateral_bound, ...
        upper_z, lower_z);
    [pthObj, solnInfo] = rrt_stl(qrss, sv, startPose, goalPose, 4000, 20, omap, ...
        1, upper_z, lower_z, lateral_bound, 20);
    robustness_matrix(c,3) = robustnessCalculator(pthObj.States, omap, lateral_bound, ...
        upper_z, lower_z);
    [pthObj, solnInfo] = rrt_stl(qrss, sv, startPose, goalPose, 4000, 20, omap, ...
        .5, upper_z, lower_z, lateral_bound, 20);
    robustness_matrix(c,3) = robustnessCalculator(pthObj.States, omap, lateral_bound, ...
        upper_z, lower_z);



end

% disp("robustness for threshold 1: ");
% disp(robustness);
% plotting_data = [];
% starting_thresh = 1;
% ending_thresh = 11;
% num_neighbors_start = 5;
% num_neighbors_end = 10;
% matrix_data = zeros(num_neighbors_end-num_neighbors_start+1,11);
% 
% for j = num_neighbors_start:num_neighbors_end
%     for i = starting_thresh:1:ending_thresh
%         [pthObj, solnInfo] = rrt_stl(qrss, sv, startPose, goalPose, 4000, 20, omap, ...
%         i/10, upper_z, lower_z, lateral_bound, j);
%         
%         robustness = robustnessCalculator(pthObj.States, omap, lateral_bound, ...
%         upper_z, lower_z);
%         
%         %fprintf('robustness for threshold %d and number of neighbors %d: %.2f\n', i, j, robustness);
%         %disp(robustness);
%         plotting_data = [plotting_data; [robustness, i, j]];
%         matrix_data(j-4,i) = robustness;
%     end
%     disp(j);
% end
% [X,Y] = meshgrid(starting_thresh:ending_thresh,num_neighbors_start:num_neighbors_end);
%X = num_neighbors_start:num_neighbors_end;
%Y = starting_thresh:ending_thresh;
%surf(X, Y, matrix_data);
%scatter3(plotting_data(:,3), plotting_data(:,2), plotting_data(:,1), "filled");
%writematrix(plotting_data, 'plotted_data.xls');

%     %% Plot
%     if (solnInfo.IsPathFound)
%         figure("Name","OriginalPath")
%         % Visualize the 3-D map
%         show(omap)
%         hold on
%         scatter3(startPose(1),startPose(2),startPose(3),30,"red","filled")
%         scatter3(goalPose(1),goalPose(2),goalPose(3),30,"green","filled")
%         
%         hReference = plot3(pthObj.States(:,1), ...
%             pthObj.States(:,2), ...
%             pthObj.States(:,3), ...
%             "LineWidth",2,"Color","g");
%       
%         legend(hReference,"Reference","Location","best")
%         hold off
%         view([-31 63])
%     end
%end

%% Functions


function [pthObj, solnInfo] = rrt_stl(ss, sv, startPose, goalPose, ...
    max_iter, step_size, omap, threshold, upper_z, lower_z, lateral_bound, num_neighbors)
    
    startingNode = RRT_Node(startPose, NaN);
    startingNode.hasParent = 0;
    goalNode = RRT_Node(goalPose, NaN);
    goalNode.hasParent = 0;
    path = [];
    solnInfo.IsPathFound = 0;


    for k = 1:max_iter
        % random sample
        sample = zeros(1,ss.NumStateVariables);
        for i = 1:ss.NumStateVariables
            if i <= 3
                sample(i) = randsample(ss.StateBounds(i,2),1);
            end
        end
    
        % get 10 nearest neighbors
        bias = rand();
        [neighbors] = biasStep(startingNode, sample, k, bias, threshold, num_neighbors);
        saved_new_nodes = [];
        null_node_state = [0, 0, 0, 0, 0, 0, 0];

        % extend sample, check for completion, and trace back
        
        robustnessArr = [];
        for i= 1:size(neighbors,2)
            unit_v = (sample-neighbors(i).state)/norm(sample-neighbors(i).state);
            new_node_state = neighbors(i).state + step_size*unit_v;
            
            
            if isStateValid(sv, new_node_state) && isMotionValid(sv, neighbors(i).state, new_node_state)
                saved_new_nodes = [saved_new_nodes; new_node_state];

               
                possible_path = [new_node_state];
%                current = neighbors(i);
%                 while current.hasParent
%                     possible_path = [possible_path; current.state];
%                     current = current.parent;
%                 end
                %add start node since there is no do-while in matlab
                %possible_path = [possible_path; startPose];
                robustnessVal = robustnessCalculator(possible_path, omap, lateral_bound, upper_z, lower_z);
                robustnessArr = [robustnessArr; robustnessVal];
            else
                saved_new_nodes = [saved_new_nodes; null_node_state];
            end
        end

        % scan robustness array
        max_robust_index = 1;
        max_robustness_val = -inf;
        for x = 1:size(robustnessArr)
            if max_robustness_val < robustnessArr(x)
                max_robust_index = x;
            end
        end


        % add new node
        new_node = RRT_Node(saved_new_nodes(max_robust_index, :), neighbors(max_robust_index));

        neighbors(max_robust_index).children = horzcat(neighbors(max_robust_index).children, new_node);
        if norm(goalPose - new_node.state) <= step_size
            goalNode.parent = new_node;
            new_node.children = horzcat(new_node.children,goalNode);
            goalNode.hasParent = 1;
            current = goalNode;
            while current.hasParent
                path = [path; current.state];
                current = current.parent;
            end
            %add start node since there is no do-while in matlab
            path = [path; startPose];
            solnInfo.IsPathFound = 1;
            path = flip(path);
            break
        end

        for i= 1:size(neighbors,2)
            neighbors(i).dist = 0;
        end
    end
    
    pthObj.States = path;
end

function [robustnessValue] = robustnessCalculator(path, omap, lateral_bound, upper_z, lower_z)

    u_z_min = inf;
    l_z_min = inf;

    [distance_arr] = distanceCalculator(path, omap, lateral_bound);

    shortest_distance = min(distance_arr);

    for l = 1:size(path,1)
        if u_z_min > upper_z - path(l,3)
            u_z_min = upper_z - path(l,3);
        end

        if l_z_min > path(l,3) - lower_z
            l_z_min = path(l,3) - lower_z;
        end
    end
    vertical_robustness = u_z_min + l_z_min;
    robustnessValue = vertical_robustness + shortest_distance;
    %robustnessValue = min([shortest_distance, l_z_min, u_z_min]);
end



function [distance_arr] = distanceCalculator(possible_path, omap, lateral_bound)
    distance_arr = [];        
    numRays = 10;
    angles = linspace(-2*pi,2*pi,numRays);

    directions_pitch = [cos(angles); zeros(1,numRays); sin(angles)]';
    directions_roll = [zeros(1,numRays); sin(angles); cos(angles)]';
    directions_yaw = [cos(angles); sin(angles); zeros(1,numRays)]';
    maxrange = 50;
    
    for d = 1:size(possible_path,1)
        
        min_ray_arr = [];

        [intersectionPts_p,isOccupied_p] = rayIntersection(omap,possible_path(d,:),directions_pitch,maxrange);
        [intersectionPts_r,isOccupied_r] = rayIntersection(omap,possible_path(d,:),directions_roll,maxrange);
        [intersectionPts_y,isOccupied_y] = rayIntersection(omap,possible_path(d,:),directions_yaw,maxrange);

        min_ray_arr = [min_ray_arr; distanceRobustness(possible_path, intersectionPts_p, numRays, isOccupied_p, lateral_bound, d)];
        min_ray_arr = [min_ray_arr; distanceRobustness(possible_path, intersectionPts_r, numRays, isOccupied_r, lateral_bound, d)];
        min_ray_arr = [min_ray_arr; distanceRobustness(possible_path, intersectionPts_y, numRays, isOccupied_y, lateral_bound, d)];
        distance_arr = [distance_arr; min(min_ray_arr)];
    end
end




function [min_ray_dist] = distanceRobustness(possible_path, intersectionPts, numRays, isOccupied, lateral_bound, index)
min_ray_dist = 50;
    for n = 1:numRays
        if isOccupied(n)
            ray_dist = norm(intersectionPts(n, :) - [possible_path(index,1), possible_path(index,2), possible_path(index,3)]) - lateral_bound;
            if ray_dist < min_ray_dist
                min_ray_dist = ray_dist;
            end
        end
    end
end


function [neighbors] = biasStep(startingNode, sample, k, bias, threshold, num_neighbors)
    neighbors = [];
    dist = inf;
    nearest_neighbor = startingNode;
    nodelist = [startingNode];
    while size(nodelist) > 0
        possible_node = nodelist(end);
        nodelist = nodelist(1:end-1);
        nodelist = horzcat(nodelist, possible_node.children);
        node_dist = norm(possible_node.state - sample);
        if k>=num_neighbors && bias >= threshold
            if(size(neighbors,2) < num_neighbors)
                possible_node.dist = node_dist;
                temp = horzcat(neighbors, possible_node);
                [~,ind] = sort([temp.dist]);
                neighbors = temp(ind);
            else
                if node_dist < neighbors(end).dist
                    neighbors = neighbors(1:end-1);
                    possible_node.dist = node_dist;
                    temp = horzcat(neighbors, possible_node);
                    [~,ind] = sort([temp.dist]);
                    neighbors = temp(ind);
                end
            end
        else
            if node_dist < dist
                dist = node_dist;
                neighbors = [possible_node];
            end
        end
    end
end

%     if k>=10 && bias >= threshold
%         nodelist = [startingNode];
%         while size(nodelist) > 0
%             possible_node = nodelist(end);
%             nodelist = nodelist(1:end-1);
%             nodelist = horzcat(nodelist, possible_node.children);
%             node_dist = norm(possible_node.state - sample);
%             if(size(neighbors,2) < 10)
%                 possible_node.dist = node_dist;
%                 temp = horzcat(neighbors, possible_node);
%                 [~,ind] = sort([temp.dist]);
%                 neighbors = temp(ind);
%             else
%                 if node_dist < neighbors(end).dist
%                     neighbors = neighbors(1:end-1);
%                     possible_node.dist = node_dist;
%                     temp = horzcat(neighbors, possible_node);
%                     [~,ind] = sort([temp.dist]);
%                     neighbors = temp(ind);
%                 end
%             end
%         end
%     else
%         nodelist = [startingNode];
%         while size(nodelist) > 0
%             possible_node = nodelist(end);
%             nodelist = nodelist(1:end-1);
%             nodelist = horzcat(nodelist, possible_node.children);
%             node_dist = norm(possible_node.state - sample);
%             if node_dist < dist
%                 dist = node_dist;
%                 nearest_neighbor = possible_node;
%             end
%         end
%         neighbors = [nearest_neighbor];
%     end
% end
    

     %   unit_v = (sample-nearest_neighbor.state)/norm(sample-nearest_neighbor.state);
      %  new_node_state = nearest_neighbor.state + step_size*unit_v;
      %  if isStateValid(sv, new_node_state)
      %      new_node = RRT_Node(new_node_state, nearest_neighbor);
      %      nearest_neighbor.children = horzcat(nearest_neighbor.children, new_node);
      %      if norm(goalPose - new_node_state) <= step_size
      %          goalNode.parent = new_node;
      %          new_node.children = horzcat(new_node.children,goalNode);
      %          goalNode.hasParent = 1;
      %          current = goalNode;
      %          while current.hasParent
      %              path = [path; current.state];
      %              current = current.parent;
      %          end
      %          %add start node since there is no do-while in matlab
      %          path = [path; startPose];
      %          solnInfo.IsPathFound = 1;
      %          path = flip(path);
      %          break
      %      end
      %  end


%function [robustness] = check_stl(new_node_state)
%    path = [new_node_state];
%    current = neighbors(i);
%    while current.hasParent
%        path = [path; current.state];
%        current = current.parent;
%    end
%    path = [path; startPose];
 %   path = flip(path);
    %check stl
%end
