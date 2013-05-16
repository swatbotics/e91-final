dat = importdata('timing.txt');

freq = 2;

time = dat.data(:,1);
time = time./freq;

%% plot
figure(1)
subplot(2,2,1);
plot(time, dat.data(:,2));
title('x zmp');
xlabel('time ms');
subplot(2,2,2);
plot(time, dat.data(:,3));
title('y zmp');
xlabel('time ms');
subplot(2,2,3);
plot(time, dat.data(:,4));
title('stance');
xlabel('time ms');