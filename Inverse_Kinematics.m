%Opendog project
%Inverse kinematics calculation
%Sanha Lee

%calculation is based on the document https://docs.google.com/document/d/15PCJ9cBs33W3WM1cPE70XX0P2loeLgb8bcA4J7JWhHM/edit

clc
clear all
close all

%Define constants


x= [1.75 4.625 8.5 11.0 13.5 14.5 9.5625 5.75 1.75];  %current x position of a foot
y=[24.5 22.125 20.9 20.695 20.78 24.5 24.5 24.5 24.5]; %current y position of a foot

lf= 14.107;         %length of the upper leg in inch
lt= 12.233;         %length of lower leg in inch


%distance between the hip joint and the foot
d=sqrt((x.^2+(y-3).^2));

%finding angles
theta_p = atan(y./x);

theta_k = acos((lf.^2 + d.^2 - lt.^2)/(2.*d.*lf)) *(180/pi);

theta_hkp = acos((12.645^2+14.015^2-d.^2)/(2.*12.645.*14.015)) * (180/pi); 

theta_h = acos(3.872/4.047) *(180/pi); 

theta_k_shift=acos((10.432^2 + 14.283^2 - 4.669^2)/(2*10.432*14.283))*(180/pi);

theta_hkp_shift = acos((3.759^2 + 13.42^2 - 9.662^2)/(2*3.759*13.42))*(180/pi);

theta_t_shift= acos((10.433^2 + 14.941^2 - 5.01^2)/(2*10.433*14.941))*(180/pi);

theta_f = theta_p - theta_k - theta_k_shift + theta_h

theta_t = theta_hkp - theta_hkp_shift - theta_t_shift


