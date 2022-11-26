%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                   %
%   Dijkstra from image                                             %
%                                    Arrigoni 11.2022 v .1          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% get image and convert to OccupancyGrid
clc
clear all
close all

%%
map_rgb = imread('mappa_test_red.png');

BW = im2bw(map_rgb,0.95);

% imshow(BW)
% plot_map(BW)

% initialize variables
G=-1*ones(size(BW,1)*size(BW,2)); % tree dependencies
dist=inf*ones(size(BW,1)*size(BW,2),1);
prec=inf*ones(size(BW,1)*size(BW,2),1); 
nodelist=-1*ones(size(BW,1)*size(BW,2),1); % to visit

% create edge matrix

for ii=1:size(BW,1)
    for j=1:size(BW,2)
        if BW(ii,j)==1 
            %%% itself
            G((ii-1)*size(BW,1)+j,(ii-1)*size(BW,1)+j)=0;

            %%% 4 directions
            if ii+1 >0 && ii+1<=size(BW,1) && BW(ii+1,j)==1
                G((ii-1)*size(BW,1)+j,(ii+1-1)*size(BW,1)+j)=1;
            end
            if ii-1 >0 && ii-1<=size(BW,1) && BW(ii-1,j)==1
                G((ii-1)*size(BW,1)+j,(ii-1-1)*size(BW,1)+j)=1;
            end
            if j+1 >0 && j+1<=size(BW,2) && BW(ii,j+1)==1
                G((ii-1)*size(BW,1)+j,(ii-1)*size(BW,1)+j+1)=1;
            end
            if j-1 >0 && j-1<=size(BW,2) && BW(ii,j-1)==1
                G((ii-1)*size(BW,1)+j,(ii-1)*size(BW,1)+j-1)=1;
            end
        end
    end
end


%% initialize system:

start_pos = [1,1];
goal_pos = [20,30];

start= (start_pos(2)-1)*size(BW,1)+start_pos(1);
goal= (goal_pos(2)-1)*size(BW,1)+goal_pos(1);


%%% visualize map start & goal (pcolor?)

plot_map(BW)

plot(start_pos(1),start_pos(2),'sg','MarkerFaceColor','g')
plot(goal_pos(1),goal_pos(2),'sr','MarkerFaceColor','r')

%%%% Dijkstra %%%%%%%%%%%%%%%%%%%%
%%% initialize actual node = start

dist(start)=0; % inital distance
act_node=start;

[~,con_nodes]=find(G(act_node,:)>0); % def connected vertex
nodelist(con_nodes,1) = 1; % add to visit
nodelist(act_node,1)=0; % visited
%%% further steps

while any(nodelist(:,1)==1) && act_node~=goal
    i_con=length(con_nodes);
    while i_con>0 % avail & not visited
        if dist(con_nodes(i_con))>dist(act_node)+1 % if not measured or shorter
            dist(con_nodes(i_con))= dist(act_node)+1;% calc dist for the new node
            prec(con_nodes(i_con))=act_node;
        end
        i_con=i_con-1;
    end
    
    %%% plot run-time
    plot_runtime(act_node,BW)
    
    %%% evaluate new candidate node & new neighbours
    [min_val,~]=min(dist(nodelist(:,1)==1));
    new_nodes=find(dist==min_val);
    tmp_i=1;
    while nodelist(new_nodes(tmp_i),1)~=1 
        tmp_i=tmp_i+1; % first best
    end    
    act_node=new_nodes(tmp_i); % new node
    nodelist(act_node,1)=0; % visited
    [~,con_nodes]=find(G(act_node,:)>0); % def connected vertex
    i_con=length(con_nodes);
    while i_con>0
        if nodelist(con_nodes(i_con),1) ~= 0 % if not visited
            nodelist(con_nodes(i_con),1) = 1; % add to visit
        end
        i_con=i_con-1;
    end
end


%%% shortest path
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
