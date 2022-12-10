function plot_map(BW)

figure()
hold on
for ii=1:size(BW,1)
    for jj=1:size(BW,2)
        if BW(ii,jj)==1
            plot(jj,ii,'oc', 'MarkerFaceColor','c') 
        else
            plot(jj,ii,'ok','MarkerFaceColor','k')
        end
    end
end

axis ij