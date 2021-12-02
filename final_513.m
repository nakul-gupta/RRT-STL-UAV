%% Set RNG seed for repeatable result
rng(1,"twister");

mapData = load("uavMapCityBlock.mat","omap");
omap = mapData.omap;
% Consider unknown spaces to be unoccupied
omap.FreeThreshold = omap.OccupiedThreshold;
%With or without inflate?
inflate(omap,1)

startPose = [12 22 60 0 0 0 1];
goalPose = [150 180 100 0 0 0 1];
%goalPose = [150 40 100 0 0 0 1];
figure("Name","StartAndGoal")
hMap = show(omap);
hold on
scatter3(hMap,startPose(1),startPose(2),startPose(3),30,"red","filled")
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

[pthObj, solnInfo] = rrt_stl(qrss, sv, startPose, goalPose, 4000, 20, omap);
%planner = plannerRRT(qrss,sv);
%planner.MaxConnectionDistance = 50;
%planner.GoalBias = 0.10;  
%planner.MaxIterations = 400;
%planner.GoalReachedFcn = @(~,x,y)(norm(x(1:3)-y(1:3)) < 5);

%[pthObj,solnInfo] = plan(planner,startPose,goalPose);
for i = 1:size(path)-1

    isValid = isMotionValid(sv, path(i).state, path(i+1).state);
    if isValid == 0
        disp("error");
    end
end

if (solnInfo.IsPathFound)
    figure("Name","OriginalPath")
    % Visualize the 3-D map
    show(omap)
    hold on
    scatter3(startPose(1),startPose(2),startPose(3),30,"red","filled")
    scatter3(goalPose(1),goalPose(2),goalPose(3),30,"green","filled")
    
    hReference = plot3(pthObj.States(:,1), ...
        pthObj.States(:,2), ...
        pthObj.States(:,3), ...
        "LineWidth",2,"Color","g");
  
    legend(hReference,"Reference","Location","best")
    hold off
    view([-31 63])
end

%% Functions


function [pthObj, solnInfo] = rrt_stl(ss, sv, startPose, goalPose, max_iter, step_size, omap)
    
    startingNode = RRT_Node(startPose, NaN);
    startingNode.hasParent = 0;
    goalNode = RRT_Node(goalPose, NaN);
    goalNode.hasParent = 0;
    path = [];
    total_nodes = 1;
    solnInfo.IsPathFound = 0;
    %when using a value of 8, we break through objects
    threshold = 8;
    upper_z = 150;
    lower_z = 50;
    lateral_bound = 20;

    for k = 1:max_iter
        %% random sample
        sample = zeros(1,ss.NumStateVariables);
        for i = 1:ss.NumStateVariables
            if i <= 3
                sample(i) = randsample(ss.StateBounds(i,2),1);
            end
        end
    
        %% get 10 nearest neighbors
        bias = randsample(11, 1);
        [neighbors] = biasStep(startingNode, sample, k, bias, threshold);

        %% extend sample, check for completion, and trace back
        
        robustnessArr = [];
        shortest_length = inf;
        max_vert_robustness = -inf;
        for i= 1:size(neighbors,2)
            
            u_z_min = inf;
            l_z_min = inf;
            unit_v = (sample-neighbors(i).state)/norm(sample-neighbors(i).state);
            new_node_state = neighbors(i).state + step_size*unit_v;
            
            if isStateValid(sv, new_node_state) == 0
            %if checkOccupancy(omap, [new_node_state(1),new_node_state(2),new_node_state(3)])
               
                possible_path = [new_node_state];
                current = neighbors(i);
                while current.hasParent
                    possible_path = [possible_path; current.state];
                    current = current.parent;
                end
                %add start node since there is no do-while in matlab
                possible_path = [possible_path; startPose];
                possible_path = flip(possible_path);
                
                

                %robust calculator
                %if size(possible_path,1) < shortest_length
                %    shortest_length = size(possible_path,1);
                    %max_robust_index = i;
                    
                %end

                for l = 1:size(possible_path,1)
                    if u_z_min > upper_z - possible_path(l,3)
                        u_z_min = upper_z -possible_path(l,3);
                    end

                    if l_z_min > possible_path(l,3) - lower_z
                        l_z_min = possible_path(l,3) - lower_z;
                    end
                end
                vertical_robustness = u_z_min + l_z_min;
%                 if vertical_robustness > max_vert_robustness
%                        max_vert_robustness = vertical_robustness;
%                        max_robust_index = i;
%                 end
                shortest_length = size(possible_path,1);
                robustnessArr = [robustnessArr; vertical_robustness - shortest_length];
            end
        end

        %% scan robustness array
        max_robust_index = 1;
        max_robustness_val = -inf;
        for x = 1:size(robustnessArr)
            if max_robustness_val < robustnessArr(x)
                max_robust_index = x;
            end
        end


        %% add new node
        unit_v = (sample-neighbors(max_robust_index).state)/norm(sample-neighbors(max_robust_index).state);
        %new_node_state = neighbors(i).state + step_size*unit_v;
        %neighbors(max_robust_index).state + step_size*unit_v
        new_node = RRT_Node(neighbors(max_robust_index).state + step_size*unit_v, neighbors(max_robust_index));
%         isValid = isStateValid(sv, new_node.state);
%         if isValid == 0
%             disp("error");
%         end
        isValid = checkOccupancy(omap, [new_node.state(1),new_node.state(2),new_node.state(3)]);
        if isValid == 1
            disp("error");
        end

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
            isValid = isStateValid(sv, path);
            if isValid == 0
                disp("error");
            end
            break
        end
        %else
%             unit_v = (sample-nearest_neighbor.state)/norm(sample-nearest_neighbor.state);
%             new_node_state = nearest_neighbor.state + step_size*unit_v;
%             if isStateValid(sv, new_node_state)
%                 new_node = RRT_Node(new_node_state, nearest_neighbor);
%                 nearest_neighbor.children = horzcat(nearest_neighbor.children, new_node);
%                 if norm(goalPose - new_node_state) <= step_size
%                     goalNode.parent = new_node;
%                     new_node.children = horzcat(new_node.children,goalNode);
%                     goalNode.hasParent = 1;
%                     current = goalNode;
%                     while current.hasParent
%                         path = [path; current.state];
%                         current = current.parent;
%                     end
%                     %add start node since there is no do-while in matlab
%                     path = [path; startPose];
%                     solnInfo.IsPathFound = 1;
%                     path = flip(path);
%                     break
%                 end
%             end
        %end
        for i= 1:size(neighbors,2)
            neighbors(i).dist = 0;
        end
    end
    
    pthObj.States = path;
end

function [neighbors] = biasStep(startingNode, sample, k, bias, threshold)
    neighbors = [];
    dist = inf;
    nearest_neighbor = startingNode;
    nodelist = [startingNode];
    while size(nodelist) > 0
        possible_node = nodelist(end);
        nodelist = nodelist(1:end-1);
        nodelist = horzcat(nodelist, possible_node.children);
        node_dist = norm(possible_node.state - sample);
        if k>=10 && bias >= threshold
            if(size(neighbors,2) < 10)
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
