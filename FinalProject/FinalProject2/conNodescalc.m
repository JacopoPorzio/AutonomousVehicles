function [conNodes, costNodes] = conNodescalc(BW, q)

ny = fix(q/size(BW, 1)) +  1*(mod(q, size(BW, 1)) ~= 0);
nx = q - (ny - 1)*size(BW, 1);

% do not reduce image! keep it as it is, in order to know the nodes

% find all the connected nodes (i.e. also the inaccessible ones

conNodesMax = -1*ones(1, 8);
costNodesMax = -1*ones(1, 8);

% G = -1*ones(1, 9);

if BW(ny, nx) == 1
    %%% itself
%     G( (ny - 1)*size(BW, 1) + nx,(ny - 1)*size(BW, 1) + nx) = 0;


    %%% 4 directions
    %
    BCon = ny + 1 > 0 && ny + 1 <= size(BW, 1) && BW(ny + 1, nx) == 1;
    ACon = ny - 1 > 0 && ny - 1 <= size(BW, 1) && BW(ny - 1, nx) == 1;
    RCon = nx + 1 > 0 && nx + 1 <= size(BW, 2) && BW(ny, nx + 1) == 1;
    LCon = nx - 1 > 0 && nx - 1 <= size(BW, 2) && BW(ny, nx - 1) == 1;

    % below
    if BCon
%         G((ny - 1)*size(BW, 1) + nx, (ny + 1 - 1)*size(BW, 1) + nx) = 1;
        conNodesMax(1, 7) = (ny + 1 - 1)*size(BW, 1) + nx;
        costNodesMax(1, 7) = 1;
    end

    % above
    if ACon
%         G((ny - 1)*size(BW, 1) + nx, (ny - 1 - 1)*size(BW, 1) + nx) = 1;
        conNodesMax(1, 2) = (ny - 1 - 1)*size(BW, 1) + nx;
        costNodesMax(1, 2) = 1;
    end

    % right
    if RCon
%         G((ny - 1)*size(BW, 1) + nx, (ny - 1)*size(BW, 1) + nx + 1) = 1;
        conNodesMax(1, 5) = (ny - 1)*size(BW, 1) + (nx + 1);
        costNodesMax(1, 5) = 1;
    end

    % left
    if LCon
%         G((ny - 1)*size(BW, 1) + nx, (ny - 1)*size(BW, 1) + nx - 1) = 1;
        conNodesMax(1, 4) = (ny - 1)*size(BW, 1) + (nx - 1);
        costNodesMax(1, 4) = 1;
    end

    %%% exploring diagonals
    %
    % NW \|/ NE
    %  ---+---
    % SW /|\ SE
    %
    SWCon = (ny + 1 > 0 && nx - 1 > 0) && (ny + 1 <= size(BW, 1) && nx - 1 <= size(BW, 2)) && (BW(ny + 1, nx - 1) == 1);
    NWCon = (ny - 1 > 0 && nx - 1 > 0) && (ny - 1 <= size(BW, 1) && nx - 1 <= size(BW, 2)) && (BW(ny - 1, nx - 1) == 1);
    SECon = (ny + 1 > 0 && nx + 1 > 0) && (ny + 1 <= size(BW, 1) && nx + 1 <= size(BW, 2)) && (BW(ny + 1, nx + 1) == 1);
    NECon = (ny - 1 > 0 && nx + 1 > 0) && (ny - 1 <= size(BW, 1) && nx + 1 <= size(BW, 2)) && (BW(ny - 1, nx + 1) == 1);


    % NE
    if NECon
%         G((ny - 1)*size(BW, 1) + nx, (ny - 1 - 1)*size(BW, 1) + nx + 1) = sqrt(2);
        conNodesMax(1, 3) = (ny - 1 - 1)*size(BW, 1) + (nx + 1);
        costNodesMax(1, 3) = sqrt(2);
    end

    % NW
    if NWCon
%         G((ny - 1)*size(BW, 1) + nx, (ny - 1 - 1)*size(BW, 1) + nx - 1) = sqrt(2);
        conNodesMax(1, 1) = (ny - 1 - 1)*size(BW, 1) + (nx - 1);
        costNodesMax(1, 1) = sqrt(2);
    end

    % SE
    if SECon
%         G((ny - 1)*size(BW, 1) + nx, (ny + 1 - 1)*size(BW, 1) + nx + 1) = sqrt(2);
        conNodesMax(1, 8) = (ny + 1 - 1)*size(BW, 1) + (nx + 1);
        costNodesMax(1, 8) = sqrt(2);
    end

    % SW
    if SWCon
%         G((ny - 1)*size(BW, 1) + nx, (ny + 1 - 1)*size(BW, 1) + nx - 1) = sqrt(2);
        conNodesMax(1, 6) = (ny + 1 - 1)*size(BW, 1) + (nx - 1);
        costNodesMax(1, 6) = sqrt(2);
    end


end

conNodes = conNodesMax(conNodesMax > 0);
costNodes = costNodesMax(costNodesMax > 0);
