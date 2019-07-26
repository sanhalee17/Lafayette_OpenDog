%Maximum and minimum angla and length calculation

%OpenDog Project
%7/24/2019

%Sanha Lee

%Defining constant
%unit: in

x_min = 6.875;
y_min = 16.625;

x_max = 10;
y_max = 25.75;

%distance calculation
min_length = sqrt(x_min ^2 + y_min^2) 
max_length = sqrt(x_max ^2 + y_max^2)

%extended leg angle calculation
max_angle_E = atan(11.5/13.75)*(180/pi) %degree
min_angle_E = atan(23.625/13.25)*(180/pi)


%contracted leg angle culation
max_angle_C = atan(1.25/17.875) *(180/pi)
min_angle_C = atan(17.25/5.875) *(180/pi)