%% Set RNG seed for repeatable result
rng(1,"twister");

mapData = load("uavMapCityBlock.mat","omap");
omap = mapData.omap;
% Consider unknown spaces to be unoccupied
omap.FreeThreshold = omap.OccupiedThreshold;
%With or without inflate?
inflate(omap,1)

startPose = [12 22 100 0 0 0 1];
goalPose = [150 180 100 0 0 0 1];
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
sv.ValidationDistance = 0.1;

[pthObj, solnInfo] = rrt_stl(qrss, sv, startPose, goalPose, 400, 100);
%planner = plannerRRT(qrss,sv);
%planner.MaxConnectionDistance = 50;
%planner.GoalBias = 0.10;  
%planner.MaxIterations = 400;
%planner.GoalReachedFcn = @(~,x,y)(norm(x(1:3)-y(1:3)) < 5);

%[pthObj,solnInfo] = plan(planner,startPose,goalPose);

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


function [pthObj, solnInfo] = rrt_stl(ss, sv, startPose, goalPose, max_iter, step_size)
    
    startingNode = RRT_Node(startPose, NaN);
    startingNode.hasParent = 0;
    goalNode = RRT_Node(goalPose, NaN);
    goalNode.hasParent = 0;
    path = [];
    total_nodes = 1;
    solnInfo.IsPathFound = 0;

    %disp(startingNode);

    for k = 1:max_iter

        %% random sample
        sample = zeros(1,ss.NumStateVariables);
        for i = 1:ss.NumStateVariables
            if i <= 3
                sample(i) = randsample(ss.StateBounds(i,2),1);
            end
        end
    
        %% get 10 nearest neighbors
        %neighbors = RRT_Node.empty(10,0);
        neighbors = [];
        if k>=10
            dist = norm(startingNode.state - sample);
            nearest_neighbor = startingNode;
            nodelist = [startingNode];
            while size(nodelist) > 0
                possible_node = nodelist(end);
                nodelist = nodelist(1:end-1);
                nodelist = horzcat(nodelist, possible_node.children);
                node_dist = norm(possible_node.state - sample);
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
            end
        else
            dist = norm(startingNode.state - sample);
            nearest_neighbor = startingNode;
            nodelist = [startingNode];
            while size(nodelist) > 0
                possible_node = nodelist(end);
                nodelist = nodelist(1:end-1);
                nodelist = horzcat(nodelist, possible_node.children);
                node_dist = norm(possible_node.state - sample);
                if node_dist < dist
                    dist = node_dist;
                    nearest_neighbor = possible_node;
                end
            end
        end
       
       
            

        %get nearest neighbor
        %dist = norm(startingNode.state - sample);
        %nearest_neighbor = startingNode;
        %nodelist = [startingNode];
        %while size(nodelist) > 0
        %    possible_node = nodelist(end);
        %    nodelist = nodelist(1:end-1);
        %    nodelist = horzcat(nodelist, possible_node.children);
        %    node_dist = norm(possible_node.state - sample);
        %    if node_dist < dist
        %        dist = node_dist;
        %        nearest_neighbor = possible_node;
        %    end
        %end

        %extend sample, check for completion, and trace back
        max_robust_index = 1;
        shortest_length = inf;
        if k >= 10
            for i= 1:size(neighbors,2)
                unit_v = (sample-neighbors(i).state)/norm(sample-neighbors(i).state);
                new_node_state = neighbors(i).state + step_size*unit_v;
                if isStateValid(sv, new_node_state)
                    possible_path = [new_node_state];
                    current = neighbors(i);
                    while current.hasParent
                        possible_path = [possible_path; current.state];
                        current = current.parent;
                    end
                    %add start node since there is no do-while in matlab
                    possible_path = [possible_path; startPose];
                    possible_path = flip(possible_path);
                    if size(possible_path,1) < shortest_length
                        shortest_length = size(possible_path,1);
                        max_robust_index = i;
                    end
                end
            end
            new_node = RRT_Node(new_node_state, neighbors(max_robust_index));
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
        else
            unit_v = (sample-nearest_neighbor.state)/norm(sample-nearest_neighbor.state);
            new_node_state = nearest_neighbor.state + step_size*unit_v;
            if isStateValid(sv, new_node_state)
                new_node = RRT_Node(new_node_state, nearest_neighbor);
                nearest_neighbor.children = horzcat(nearest_neighbor.children, new_node);
                if norm(goalPose - new_node_state) <= step_size
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
            end
        end
    end
    
    pthObj.States = path;
end
    

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