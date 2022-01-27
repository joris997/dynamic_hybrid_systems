clear; close; clc;

in = [1 1 0 1 1 0 1 1;
      0 1 1 0 0 0 0 0;
      0 0 1 0 0 1 0 0;
      0 0 0 0 1 0 1 0;
      0 1 1 1 1 1 1 0;
      0 1 0 0 1 0 0 1];
  
out = [];
for i = 1:size(in,1)
    inTS = [linspace(0,7,8)' in(i,:)'];
    outTS = sim('Mealy',7);
    out = [out;
           outTS.two_2.Data'];
end

%%
figure
subplot(5,3,1)
plot(linspace(0,7,8),in(1,:),'bo')
grid on
ylabel('input [-]')
set(gca,'FontSize',10)
subplot(5,3,4)
plot(outTS.two_2.Time,out(1,:),'ro')
grid on
xlabel('timestep [-]')
ylabel('output [-]')
ylim([0 2])
set(gca,'FontSize',10)

subplot(5,3,2)
plot(linspace(0,7,8),in(2,:),'bo')
grid on
ylabel('input [-]')
set(gca,'FontSize',10)
subplot(5,3,5)
plot(outTS.two_2.Time,out(2,:),'ro')
grid on
xlabel('timestep [-]')
ylabel('output [-]')
ylim([0 2])
set(gca,'FontSize',10)

subplot(5,3,3)
plot(linspace(0,7,8),in(3,:),'bo')
grid on
ylabel('input [-]')
set(gca,'FontSize',10)
subplot(5,3,6)
plot(outTS.two_2.Time,out(3,:),'ro')
grid on
xlabel('timestep [-]')
ylabel('output [-]')
ylim([0 2])
set(gca,'FontSize',10)




subplot(5,3,10)
plot(linspace(0,7,8),in(4,:),'bo')
grid on
ylabel('input [-]')
set(gca,'FontSize',10)
subplot(5,3,13)
plot(outTS.two_2.Time,out(4,:),'ro')
grid on
xlabel('timestep [-]')
ylabel('output [-]')
ylim([0 2])
set(gca,'FontSize',10)

subplot(5,3,11)
plot(linspace(0,7,8),in(5,:),'bo')
grid on
ylabel('input [-]')
set(gca,'FontSize',10)
subplot(5,3,14)
plot(outTS.two_2.Time,out(5,:),'ro')
grid on
xlabel('timestep [-]')
ylabel('output [-]')
ylim([0 2])
set(gca,'FontSize',10)

subplot(5,3,12)
plot(linspace(0,7,8),in(6,:),'bo')
grid on
ylabel('input [-]')
set(gca,'FontSize',10)
subplot(5,3,15)
plot(outTS.two_2.Time,out(6,:),'ro')
grid on
xlabel('timestep [-]')
ylabel('output [-]')
ylim([0 2])
set(gca,'FontSize',10)

sgtitle('Mealy simulations for sounding alarm')