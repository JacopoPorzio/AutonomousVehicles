% clockStructs = {}; scanStructs = {}; odomStructs = {};
% 
% for ii = 1:length(msgStructs)
% 
%     if msgStructs{ii}.MessageType == 'sensor_msgs/LaserScan'
%         
%         scanStructs{ii} = msgStructs{ii};
% 
%     elseif msgStructs{ii}.MessageType == 'nav_msgs/Odometry'
%         
%         odomStructs{ii} = msgStructs{ii};
% 
%     elseif msgStructs{ii}.MessageType == 'rosgraph_msgs/Clock'
% 
%         clockStructs{ii} = msgStructs{ii};
% 
%     end
%     
% end

