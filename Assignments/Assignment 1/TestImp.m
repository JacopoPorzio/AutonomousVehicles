

%%
test = odomAngle(:,1);
test(2701:3029) = test(2701:3029)*(-1);
figure
plot(test)
hold on
plot(odomAngle(:,1))

