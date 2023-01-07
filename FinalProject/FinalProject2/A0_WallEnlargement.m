clear
clc
close all

%% Load
map_rgb = imread('veryFineMap2.pgm');
BW = imbinarize(map_rgb,0.60);
res = 0.05; %[m/pix]

%% PlotMap

% plot_map_mod(BW)
% xlim([0 size(BW, 1)])
% ylim([0 size(BW, 2)])
% hold on

%% Definition of needed enlargement
% maximum Turtlebot dimension is 306 (mm)

mENL = (0.5*310)*1e-3;            %[m]

eta = 3;                        %[-], safety coefficient

enl = floor(eta*mENL/res);        %[pix]

% the variable enl represents the needed augmentation

%% Enlargement

% Denl = strel('square', enl);
Denl = ones(enl, enl);

if 1 
    BWd = abs(1 - BW);
    BWe = imdilate(BWd, Denl);
    BWe = abs(1 - BWe);
else
    BWe = imerode(BW, Denl);
end

figure
hold on
for ii=1:size(BWe,1)
    for jj=1:size(BWe,2)
        if BWe(ii,jj) == 0
            plot(jj,ii,'ok','MarkerFaceColor','b', 'LineWidth', 0.1);
        end
    end
end

axis ij

hold on
for ii=1:size(BW,1)
    for jj=1:size(BW,2)
        if BW(ii,jj) == 0
            plot(jj,ii,'ok','MarkerFaceColor','k', 'LineWidth', 0.1);
        end
    end
end



xlim([0 size(BW, 1)])
ylim([0 size(BW, 2)])

