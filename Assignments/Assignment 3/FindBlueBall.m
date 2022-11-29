function [c, mag, ball1] = FindBlueBall(img, params)
    % exampleHelperTurtleBotFindBlueBall - Searches an image for a blue ball and returns its
    %   center and radius
    %   [C,M,BALL] = exampleHelperTurtleBotFindBlueBall(IMG,PARAMS) searches a color image (IMG) to
    %   find a blue ball. Threshold parameters are part of the PARAMS struct
    %   input argument and require properties of blueThresh and darkThresh. The
    %   return arguments are the center of the ball (C), the radius of the ball
    %   (M), and the binary image of the ball after thresholding (BALL).
    %
    %   Modified from TurtleBotObjectTrackingExample Arrigoni-2022
    
    %   Copyright 2014 The MathWorks, Inc.
    
    if isempty(img)
        c = [];
        mag = [];
        return
    end
    
    imSize = size(img);
    
    heightThresh = imSize(1)*0.5; % Pixel height as a function of image size
    
    % Isolate blue color by combining blue images and dark images together
    blueImg = img(:,:,1)/2 + img(:,:,2)/2 - img(:,:,3)/2;
    blueThresh = blueImg < params.blueMax;
    darkIso = -img(:,:,1)/2 - img(:,:,2)/2 + 3*img(:,:,3) - 2*rgb2gray(img);
    darkThresh = darkIso > params.darkMin;
    ball1 = blueThresh & darkThresh;
    
    % Find contiguous regions in the filtered image
    % NOTE: REGIONPROPS is a function in the Image Processing Toolbox (IPT)
    s = regionprops(ball1, {'Centroid','Area','EquivDiameter'});
    
    % If there are none, conclude that no ball exists in the image
    if isempty(s)
        c = [];
        mag = [];
    else
        % Choose the region with the largest area as the ball
        [~, id] = max([s.Area]);
        c = s(id).Centroid;
        mag = s(id).EquivDiameter/2;
    end
end