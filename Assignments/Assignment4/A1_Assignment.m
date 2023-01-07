
clc
clear
close all

%% Type of assignment

% AssignmentCase = 'DijkstraNormal';

% AssignmentCase = 'Diagonal';

% AssignmentCase = 'A*'; % it's actually a not complete A*

AssignmentCase = 'DiagonalDijkstra+A*'; % complete A*

%% Initialization of the edge matrix

% Test map
% start_pos = [1, 1];
% goal_pos = [20, 30];
% map_rgb = imread('mappa_test_red.png');

% Map 1
% start_pos = [1, 12];
% goal_pos = [28, 12];
% map_rgb = imread('map_1.png');

% Map 2
% start_pos = [1, 8];
% goal_pos = [27, 29];
% map_rgb = imread('map_2.png');

% Map 3
start_pos = [1, 1];
goal_pos = [1, 30];
map_rgb = imread('map_3.png');

% Map 4
% start_pos = [1, 8];
% goal_pos = [20, 19];
% map_rgb = imread('map_4.png');

BW = im2bw(map_rgb,0.95);

% BW = ones(3,3);

% imshow(BW)
% plot_map(BW)

% initialize variables
G           = - 1*ones(size(BW, 1)*size(BW, 2)); % tree dependencies
dist        = inf*ones(size(BW,1)*size(BW,2), 1);
prec        = inf*ones(size(BW, 1)*size(BW, 2), 1); 
nodelist    = - 1*ones(size(BW, 1)*size(BW, 2), 1); % to visit

% create edge matrix

for ii = 1:size(BW, 1)
    for jj = 1:size(BW, 2)
        if BW(ii, jj) == 1 
            %%% itself
            G( (ii - 1)*size(BW, 1) + jj,(ii - 1)*size(BW, 1) + jj)=0;


            %%% 4 directions
            %
            BCon = ii + 1 > 0 && ii + 1 <= size(BW, 1) && BW(ii + 1, jj) == 1;
            ACon = ii - 1 > 0 && ii - 1 <= size(BW, 1) && BW(ii - 1, jj) == 1;
            RCon = jj + 1 > 0 && jj + 1 <= size(BW, 2) && BW(ii, jj + 1) == 1;
            LCon = jj - 1 > 0 && jj - 1 <= size(BW, 2) && BW(ii, jj - 1) == 1;

            % right
            if BCon
                G((ii - 1)*size(BW, 1) + jj, (ii + 1 - 1)*size(BW, 1) + jj) = 1;
            end
            
            % left
            if ACon
                G((ii - 1)*size(BW, 1) + jj, (ii - 1 - 1)*size(BW, 1) + jj) = 1;
            end

            % below
            if RCon
                G((ii - 1)*size(BW, 1) + jj, (ii - 1)*size(BW, 1) + jj + 1) = 1;
            end

            % above
            if LCon
                G((ii - 1)*size(BW, 1) + jj, (ii - 1)*size(BW, 1) + jj - 1) = 1;
            end

            if strcmp(AssignmentCase, 'Diagonal') || strcmp(AssignmentCase, 'DiagonalDijkstra+A*')
                %%% exploring diagonals
                %
                % NW \|/ NE
                %  ---+---
                % SW /|\ SE
                %
                SWCon = (ii + 1 > 0 && jj - 1 > 0) && (ii + 1 <= size(BW, 1) && jj - 1 <= size(BW, 2)) && (BW(ii + 1, jj - 1) == 1);
                NWCon = (ii - 1 > 0 && jj - 1 > 0) && (ii - 1 <= size(BW, 1) && jj - 1 <= size(BW, 2)) && (BW(ii - 1, jj - 1) == 1);
                SECon = (ii + 1 > 0 && jj + 1 > 0) && (ii + 1 <= size(BW, 1) && jj + 1 <= size(BW, 2)) && (BW(ii + 1, jj + 1) == 1);
                NECon = (ii - 1 > 0 && jj + 1 > 0) && (ii - 1 <= size(BW, 1) && jj + 1 <= size(BW, 2)) && (BW(ii - 1, jj + 1) == 1);


                % NE
                if NECon
                     G((ii - 1)*size(BW, 1) + jj, (ii - 1 - 1)*size(BW, 1) + jj + 1) = sqrt(2);
                     
                end

                % NW
                if NWCon
                     G((ii - 1)*size(BW, 1) + jj, (ii - 1 - 1)*size(BW, 1) + jj - 1) = sqrt(2);
                end
    
                % SE
                if SECon
                     G((ii - 1)*size(BW, 1) + jj, (ii + 1 - 1)*size(BW, 1) + jj + 1) = sqrt(2);     
                end

                % SW
                if SWCon
                    G((ii - 1)*size(BW, 1) + jj, (ii + 1 - 1)*size(BW, 1) + jj - 1) = sqrt(2);
                end
    

            end

        end
    end

end

% figure
% heatmap(G)

%% initialize system:

start = (start_pos(2) - 1)*size(BW, 1) + start_pos(1);
goal = (goal_pos(2) - 1)*size(BW, 1) + goal_pos(1);




plot_map(BW)

plot(start_pos(1), start_pos(2), 'sg', 'MarkerFaceColor', 'g')
plot(goal_pos(1), goal_pos(2), 'sr', 'MarkerFaceColor', 'r')


dist(start) = 0; % initial distance
cur_node = start;

[~, con_nodes] = find(G(cur_node, :) > 0); % def connected vertex
nodelist(con_nodes, 1) = 1; % add to visit
nodelist(cur_node, 1) = 0; % visited
%%% further steps

while any(nodelist(:, 1) == 1) && cur_node ~= goal
    i_con = length(con_nodes);
    while i_con > 0 % avail & not visited

        switch AssignmentCase
            case 'DijkstraNormal'
                    if dist(con_nodes(i_con)) > dist(cur_node) + 1 % if not measured or shorter
                        dist(con_nodes(i_con)) = dist(cur_node) + 1; % calc dist for the new node
                        prec(con_nodes(i_con)) = cur_node;
                    end
            case 'Diagonal'
                    if ( dist(con_nodes(i_con)) > dist(cur_node) + 1 ) && G(cur_node, con_nodes(i_con)) == 1
                        dist(con_nodes(i_con)) = dist(cur_node) + 1;
                        prec(con_nodes(i_con)) = cur_node;
                    elseif ( dist(con_nodes(i_con)) > dist(cur_node) + sqrt(2)) && G(cur_node, con_nodes(i_con)) == sqrt(2)
                        dist(con_nodes(i_con)) = dist(cur_node) + sqrt(2);
                        prec(con_nodes(i_con)) = cur_node;
                    end
            case 'A*'
                   ny = fix(con_nodes(i_con)/size(BW, 1)) +  1*(mod(con_nodes(i_con), size(BW, 1)) ~= 0);
                   nx = con_nodes(i_con) - (ny - 1)*size(BW, 1);
                   CostToGo = sqrt((nx - goal_pos(1))^2 + (ny - goal_pos(2))^2);
                   if dist(con_nodes(i_con)) > dist(cur_node) + 1 + CostToGo% if not measured or shorter
                        dist(con_nodes(i_con)) = dist(cur_node) + 1 + CostToGo; % calc dist for the new node
                        prec(con_nodes(i_con)) = cur_node;
                   end
            case 'DiagonalDijkstra+A*'
                    ny = fix(con_nodes(i_con)/size(BW, 1)) +  1*(mod(con_nodes(i_con), size(BW, 1)) ~= 0);
                    nx = con_nodes(i_con) - (ny - 1)*size(BW, 1);
                    CostToGo = sqrt((nx - goal_pos(1))^2 + (ny - goal_pos(2))^2);
                    if ( dist(con_nodes(i_con)) > dist(cur_node) + 1 + CostToGo) && G(cur_node, con_nodes(i_con)) == 1
                        dist(con_nodes(i_con)) = dist(cur_node) + 1 + CostToGo;
                        prec(con_nodes(i_con)) = cur_node;
                    elseif ( dist(con_nodes(i_con)) > dist(cur_node) + sqrt(2) + CostToGo) && G(cur_node, con_nodes(i_con)) == sqrt(2)
                        dist(con_nodes(i_con)) = dist(cur_node) + sqrt(2) + CostToGo;
                        prec(con_nodes(i_con)) = cur_node;
                    end
        end

        i_con = i_con - 1;
    end
    
    %%% plot run-time
    plot_runtime(cur_node, BW)
    
    %%% evaluate new candidate node & new neighbours
    [min_val, ~] = min(dist(nodelist(:, 1) == 1));
    new_nodes = find(dist == min_val);
    tmp_i = 1;
    while nodelist(new_nodes(tmp_i), 1) ~= 1 
        tmp_i = tmp_i + 1; % first best
    end    
    cur_node = new_nodes(tmp_i); % new node
    nodelist(cur_node, 1) = 0; % visited
    [~, con_nodes] = find(G(cur_node, :) > 0); % def connected vertex
    i_con = length(con_nodes);

    while i_con > 0

        if nodelist(con_nodes(i_con),1) ~= 0 % if not visited
            nodelist(con_nodes(i_con), 1) = 1; % add to visit
        end
        i_con = i_con - 1;
    end

end


%% shortest path)

% plot_map(BW)

if dist(goal)<inf
    sol_id=goal;
    path=[];
    while(sol_id~=start)
        path=[sol_id path];
        sol_id=prec(sol_id);        
    end
    path=[sol_id path];
    %%% plot shortest path
    for ii=1:length(path)
        dr_i=0;
        dr_j=path(ii);
        while size(BW,1)<dr_j
            dr_i=dr_i+1;
            dr_j=dr_j-size(BW,1);
        end
        dr_i=(dr_i)+1;
        plot(dr_j,dr_i,'or','MarkerFaceColor','r');
    end
else 
    disp('no solution found')
end

switch AssignmentCase
            case 'DijkstraNormal'
                sgtitle('$Base\ case\ Dijkstra\ algorithm$', 'Interpreter', 'LaTex')
            case 'Diagonal'
                sgtitle('$Complete\ Dijkstra\ algorithm$', 'Interpreter', 'LaTex')
            case 'DiagonalDijkstra+A*'    
                sgtitle('$A^{*}\ algorithm$', 'Interpreter', 'LaTex')
end
ylabel('$y\ [pix]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlabel('$x\ [pix]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax1.TickLabelInterpreter = 'LaTex';
ax1.FontSize = 16;
set(gcf,'color','w');


%% Addition

nodesAnalyzed = sum(isfinite(dist));
