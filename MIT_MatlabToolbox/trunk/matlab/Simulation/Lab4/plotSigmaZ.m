% Plot Sigma Z

time = 1/200*(1:size(sigmaZ,3))';

figure
subplot(2,2,1)
plot(time, squeeze(sigmaZ(1,1,:)))
xlabel('Time (s)')
ylabel('\Sigma_Z(1,1)')

subplot(2,2,2)
plot(time, squeeze(sigmaZ(1,2,:)))
xlabel('Time (s)')
ylabel('\Sigma_Z(1,2)')

subplot(2,2,3)
plot(time, squeeze(sigmaZ(2,1,:)))
xlabel('Time (s)')
ylabel('\Sigma_Z(2,1)')

subplot(2,2,4)
plot(time, squeeze(sigmaZ(2,2,:)))
xlabel('Time (s)')
ylabel('\Sigma_Z(2,2)')