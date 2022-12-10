function plot_map_mod(BW)

% since the blue points are completely useless, they aren't plotted

figure()
hold on
for ii=1:size(BW,1)
    for jj=1:size(BW,2)
        if BW(ii,jj) == 0
            plot(jj,ii,'ok','MarkerFaceColor','k', 'LineWidth', 0.1);
        end
    end
end

axis ij