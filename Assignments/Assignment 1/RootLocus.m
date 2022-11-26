function RootLocus(ACuLoop, Td, La, kPMAX, step)

    figure
    % step = 0.1;
    kP = 0:step:kPMAX;
    LM = zeros(5, length(kP));

    for ii = 1:length(kP)
        A = ACuLoop;
        A(5, 2) = A(5, 2) - Td*kP(ii)/La;
        A(5, 4) = A(5, 4) - kP(ii)/La;
        [~,Dia] = eig(A);
        lambda = diag(Dia);
        LM(:, ii) = lambda;
%         plot(lambda, '*')
%         hold on
    end

    for ii = 1:5
        scatter(real(LM(ii,:)), imag(LM(ii,:)), [], kP, 'filled')
        hold on
    end
    colormap copper
    h = colorbar;
    ylabel(h, 'k_{P}')
    grid on
    xlabel('Re(\lambda)')
    ylabel('Im(\lambda)')
    titSTR = ['k_{P}^{max} = ', num2str(kPMAX), '. T_{d} = ' num2str(Td)];
    title(titSTR)

end