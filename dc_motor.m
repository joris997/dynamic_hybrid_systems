syms cur t vol dcur real

R = 1000;
Larray = [25, 50, 75, 100, 250, 500];

% simulate DC motor
figure
subplot(3,2,1)
vInput = [100 0 0 -100 -50];
tInput = [2 2 2 2 2];
plot([0 2 2.00001 4 4.00001 6 6.00001 8 8.00001 10], ...
    [100 100 0 0 0 0 -100 -100 -50 -50],'LineWidth',2)
grid on
xlabel('time [s]')
ylabel('voltage [V]')
title('Supplied voltage')
set(gca,'FontSize',10)

for ii = 1:length(Larray)
    L = Larray(ii);
    matlabFunction([-R*cur/L + vol/L; 0], ...
    'Vars',{t,[cur;dcur],vol}, ...
    'File', 'evaluateEoDC');

    y0 = [0 0]';
    tspan = [0 5];

    tDC = []; yDC = [];
    for i = 1:length(vInput)
        t1 = 0;
        tspan = [t1(end) tInput(i)+t1(end)];
        [t1,y1] = ode45(@(t,y)evaluateEoDC(t,y,vInput(i)),tspan,y0);
        y0 = y1(end,:);
        % add time and output to total simulation results
        try
            tDC = [tDC; t1+tDC(end)];
        catch
            tDC = t1;
        end
        yDC = [yDC; y1];
    end

    subplot(3,2,[3 5])
    plot(tDC,yDC(:,1),'LineWidth',2)
    hold on
    grid on
    xlabel('time [s]')
    ylabel('current [A]')
end
legend('25 H', '50 H', '75 H', '100 H', '250 H', '500 H')
title('Current as a function of inductance')
set(gca,'FontSize',10)





Rarray = [0, 50, 75, 100, 250, 500];
L = 100;

% simulate DC motor
for ii = 1:length(Rarray)
    R = Rarray(ii);
    matlabFunction([-R*cur/L + vol/L; 0], ...
    'Vars',{t,[cur;dcur],vol}, ...
    'File', 'evaluateEoDC');

    y0 = [0 0]';
    tspan = [0 5];

    tDC = []; yDC = [];
    for i = 1:length(vInput)
        t1 = 0;
        tspan = [t1(end) tInput(i)+t1(end)];
        [t1,y1] = ode45(@(t,y)evaluateEoDC(t,y,vInput(i)),tspan,y0);
        y0 = y1(end,:);
        % add time and output to total simulation results
        try
            tDC = [tDC; t1+tDC(end)];
        catch
            tDC = t1;
        end
        yDC = [yDC; y1];
    end

    subplot(3,2,[4 6])
    plot(tDC,yDC(:,1),'LineWidth',2)
    hold on
    grid on
    xlabel('time [s]')
    ylabel('current [A]')
end
legend('0 \Omega', '50 \Omega', '75 \Omega', '100 \Omega', '250 \Omega', '500 \Omega')
set(gca,'FontSize',10)
title('Current as a function of resistance')
