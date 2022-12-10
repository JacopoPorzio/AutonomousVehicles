% possible improvements:
% is really necessary that nodelist, prec and dist are that large?
% try with an image 900x900
% I tried making a vector = ones(3000^2, 1) and it manages to do it
% w/o problems
clc
clear
close all

%% Selection of map to use

fprintf('Which map do you want to analyse?\n')
fprintf('1: 300x300 Map 1\n')
fprintf('2: 300x300 Map 2\n')
fprintf('3: 300x300 Map 3\n')
fprintf('4: 300x300 Map 4\n')
fprintf('5: 6x6 test map without obstacles\n')
fprintf('6: 30x30 test map with obstacles\n')
fprintf('7: 300x300 test map with obstacles\n')
fprintf('8: Big test map\n')
analysedMap = input('Insert a number: ');


%% Initialization of the edge matrix
switch analysedMap
    case 5
        % % Test map 1
        start_pos = [1, 1];
        goal_pos = [6, 6];
        BW = ones(6, 6);
        BW(2:3, 2:3) = 0;
        BW(2:3, 6) = 0;
        BW = imbinarize(BW,0.95);
    case 6
        % % Test map 2
        start_pos = [1, 1];
        goal_pos = [20, 30];
        map_rgb = imread('mappa_test_red.png');
        BW = imbinarize(map_rgb,0.95);
    case 7    
        % % Big map 1
        start_pos = [1, 1];
        goal_pos = [200, 300];
        map_rgb = imread('mappa_test.png');
        BW = imbinarize(map_rgb,0.95);
    case 8
        % % big test map
        start_pos = [20, 20];
        goal_pos = [590, 590];
        map_rgb = imread('map_600.png');
        BW = imbinarize(map_rgb,0.95);
    case 1
        % % Big map 2
        start_pos = [3, 116];
        goal_pos = [273, 114];
        map_rgb = imread('map_1_d.png');
        BW = imbinarize(map_rgb,0.95);
    case 2
        % % Big map 3
        start_pos = [4, 86];
        goal_pos = [261, 280];
        map_rgb = imread('map_2_d.png');
        BW = imbinarize(map_rgb,0.95);
    case 3
        % % Big map 4
        start_pos = [3, 3];
        goal_pos = [3, 297];
        map_rgb = imread('map_3_d.png');
        BW = imbinarize(map_rgb,0.95);
    case 4
        % % Big map 5
        start_pos = [5, 72];
        goal_pos = [190, 181];
        map_rgb = imread('map_4_d.png');
        BW = imbinarize(map_rgb,0.95);
end

% plot_map(BW)
% plot_map_mod(BW)

% initialize variables
% G           = - 1*ones(size(BW, 1)*size(BW, 2)); % tree dependencies
% G           = - 1*ones(9, 9);
dist        = inf*ones(size(BW,1)*size(BW,2), 1);
prec        = inf*ones(size(BW, 1)*size(BW, 2), 1); 
nodelist    = - 1*ones(size(BW, 1)*size(BW, 2), 1); % to visit

%% initialize system:

start = (start_pos(2) - 1)*size(BW, 1) + start_pos(1);
goal = (goal_pos(2) - 1)*size(BW, 1) + goal_pos(1);

% create edge matrix
% G = g_calculation_iter(BW, start);
% figure
% heatmap(G)

[conNodes, costNodes] = conNodescalc(BW, start);

%%% visualize map start & goal (pcolor?)

% plot_map(BW)
% plot_map_mod(BW)
% 
% plot(start_pos(1), start_pos(2), 'sg', 'MarkerFaceColor', 'g')
% plot(goal_pos(1), goal_pos(2), 'sr', 'MarkerFaceColor', 'r')

%% Search algorithm
%%% initialize current node = start
tic;

dist(start) = 0; % initial distance
cur_node = start;

% [~, con_nodes] = find(G(cur_node, :) > 0); % def connected vertex
[con_nodes, ~] = conNodescalc(BW, start);
nodelist(con_nodes, 1) = 1; % add to visit
nodelist(cur_node, 1) = 0; % visited
%%% further steps

while any(nodelist(:, 1) == 1) && cur_node ~= goal

    i_con = length(con_nodes);
    [~, cost_con_nodes] = conNodescalc(BW, cur_node);

    while i_con > 0 % avail & not visited

        ny = fix(con_nodes(i_con)/size(BW, 1)) +  1*(mod(con_nodes(i_con), size(BW, 1)) ~= 0);
        nx = con_nodes(i_con) - (ny - 1)*size(BW, 1);
        CostToGo = sqrt((nx - goal_pos(1))^2 + (ny - goal_pos(2))^2);

        LinCond = cost_con_nodes(i_con) == 1;
        DiagCond = cost_con_nodes(i_con) == sqrt(2);

        if ( dist(con_nodes(i_con)) > dist(cur_node) + 1 + CostToGo) && LinCond %G(cur_node, con_nodes(i_con)) == 1
            dist(con_nodes(i_con)) = dist(cur_node) + 1 + CostToGo;
            prec(con_nodes(i_con)) = cur_node;
        elseif ( dist(con_nodes(i_con)) > dist(cur_node) + sqrt(2) + 1*0 + CostToGo) && DiagCond %G(cur_node, con_nodes(i_con)) == sqrt(2)
            dist(con_nodes(i_con)) = dist(cur_node) + sqrt(2) + CostToGo;
            prec(con_nodes(i_con)) = cur_node;
        end
           
        i_con = i_con - 1;
    end
    
    %%% plot run-time
%     plot_runtime(cur_node, BW)
    
    %%% evaluate new candidate node & new neighbours
    [min_val, ~] = min(dist(nodelist(:, 1) == 1));
    new_nodes = find(dist == min_val);
    tmp_i = 1;

    while nodelist(new_nodes(tmp_i), 1) ~= 1 
        tmp_i = tmp_i + 1; % first best
    end    

    cur_node = new_nodes(tmp_i); % new node
    nodelist(cur_node, 1) = 0; % visited
%     [~, con_nodes] = find(G(cur_node, :) > 0); % def connected vertex
    [con_nodes, ~] = conNodescalc(BW, cur_node);
    i_con = length(con_nodes);

    while i_con > 0

        if nodelist(con_nodes(i_con),1) ~= 0 % if not visited
            nodelist(con_nodes(i_con), 1) = 1; % add to visit
        end
        i_con = i_con - 1;

    end

end

toc

%% Shortest path modified

% version 1

% a nice thing to plot obstacles would be to try to find a way to
% detected their border only
% figure
% plot(0, 0)
% hold on
% plot([size(BW, 2)], [size(BW, 1)])
% axis ij

plot_map_mod(BW)

if dist(goal) < inf
    sol_id = goal;
    path = [];
    while (sol_id ~= start)
        path = [sol_id path];
        sol_id = prec(sol_id);        
    end
    path = [sol_id path];
    %%% plot shortest path
    for ii = 1:length(path)
        dr_i=0;
        dr_j=path(ii);
        while size(BW,1) < dr_j
            dr_i = dr_i + 1;
            dr_j = dr_j-size(BW, 1);
        end
        dr_i = (dr_i) + 1;
        plot(dr_j,dr_i,'or','MarkerFaceColor','r', 'LineWidth', 0.1);
    end
else 
    disp('no solution found')
end
