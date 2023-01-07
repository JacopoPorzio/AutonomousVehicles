function endSim = scan2XY(Ranges, Angles)
% Read Scan block will output all zeros until the Subscriber blocks 
% outputs a message 
endSim = any(Ranges>0);% =1 if at least one range is not zero
if endSim
    x = Ranges.*cos(Angles);
    y = Ranges.*sin(Angles);
    % only return finite values
    xValid = x(isfinite(x));
    yValid = y(isfinite(y));
    plot(xValid,yValid,marker=".",LineStyle="none", MarkerSize=8)
    title('Laser Scan')
    xlabel("X")
    ylabel("Y")
    set(gca, 'YDir','reverse')
    grid on
    axis equal
    ylim([-5 5])
    xlim([0 5])
    view([90 -90])
end