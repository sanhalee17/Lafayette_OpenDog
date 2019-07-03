% OpenDog Project
% knee angle calibration curve generation
% 7/3/2019


measured_d= [5:5:50]; %ball screw location in mm
actual_d=[0:0.01:180];
theta=[70 71 72 75 77 79 83 86 89 90]; %knee angle in degree



coeff = polyfit(measured_d, theta, 1)

calb_curve=coeff(1)*actual_d + coeff(2);

figure(1)
plot(measured_d, theta, 'o', actual_d, calb_curve,'-')

xlabel('ball screw location (mm)')
ylabel('knee angle (degree)')

