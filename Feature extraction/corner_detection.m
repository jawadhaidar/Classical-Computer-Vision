%% Reset

clc;
clear all;
close all;

%% Load the image

image=input('Please enter the name of the image (lena.jpg or checkerboard.png): ', 's');
I=imread(image);
[m n p]=size(I);
I=rgb2gray(I);
%% Parameters

sigma=input('Please enter the value of sigma (for example: 1.4): ');
window=input('Please enter the value for the window size (for example: 3): ');
constant_k = 0.1; % hyperparameter 
threshold= 500;    % found out by trial and error 

%% Develop Kernel

mask_x = [-1 0 1; -1 0 1; -1 0 1];
mask_y = mask_x';

%% Find Gradients

H_x=conv2(I,mask_x,'same');
H_y=conv2(I,mask_y,'same');

%% Harris Matrix

Gaussian = fspecial('gaussian',window,sigma);
H_x2 = conv2(H_x.^2, Gaussian, 'same');  
H_y2 = conv2(H_y.^2, Gaussian, 'same');
H_xy = conv2(H_x.*H_y, Gaussian, 'same');
harris_measure = (H_x2.*H_y2 - H_xy.^2) - constant_k*(H_x2 + H_y2).^2;
[measure index] = sort(harris_measure(:),'descend');
[row col] = ind2sub(size(harris_measure),index(1:threshold));

%% Visualize results
imshow(I);
hold on;
plot(col,row,'r*'); 
title('Detecting Corners using our method');
hold off

%% Compare
corners = detectHarrisFeatures(I,'FilterSize',3); % function from MATLAB computer vision toolbox
figure;imshow(I)
hold on
plot(corners.Location(:,1),corners.Location(:,2),'r*');
title('Detecting Corners using matlabs method');