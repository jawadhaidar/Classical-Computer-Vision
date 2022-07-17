clear all 
close all
clc

%% read image
I = imread('baboon.jpg');
I=rgb2gray(I); % convert to grayscale
figure('name','gray')
imshow(I)
sigma=1
I= imgaussfilt(I,sigma)
figure('name','blured')
imshow(I)
%% compute x and y derivative of the image
norm=(1/16)*[
     1,2,1;
     2,4,2;
     1,2,1
     ];
kx=[-1,0,1;
    -2,0,2;
    -1,0,1] ;
ky=[-1,-2,-1;
    0,0,0;
    1,2,1] ;
%Inorm = conv2(double(I), norm, 'same');
%Inorm=uint8(round(Inorm))
%imshow(Inorm)
Ix = conv2(double(I), kx, 'same');
Iy = conv2(double(I), ky, 'same');


%% Find magnitude and orientation of gradient

mag=uint8(round( sqrt(Ix.^2 + Iy.^2) ));
figure('name','magnitude')
imshow(mag)
theta=atan(Iy./Ix); % [-π/2, π/2] * I am getting some null
theta=rad2deg(theta);  % -180 , 180
% if I added 260 to the negative values, I will restore the 0-360 range
%% nonmaximum suppression 
%loop for all pixels
out=zeros(size(I,1),size(I,2))
for row=2:size(I,1)-1 % skipped first and last row
    for col=2:size(I,2)-1 % skipped first and last col
        alpha=theta(row,col);
        %first section
        if  alpha>=-22.5 && alpha<=22.5 || alpha>=157.5  || alpha<=-157.5
            %left and right
            left=mag(row,col-1);
            right=mag(row,col+1);
            if mag(row,col)>=left &&  mag(row,col)>=right
                %max
                out(row,col)=mag(row,col);
            end
        elseif alpha>22.5 && alpha<=65.5 || alpha<-112.5 && alpha>-157.5
            %diagonal pos
            if mag(row,col)>=mag(row-1,col+1) &&  mag(row,col)>=mag(row+1,col-1)
                %max
                out(row,col)=mag(row,col);
            end
        elseif alpha>65.5 && alpha<=112.5 || alpha<-65.5 && alpha>-112.5
            %vertical
            if mag(row,col)>=mag(row+1,col) &&  mag(row,col)>=mag(row-1,col)
                %max
                out(row,col)=mag(row,col);
            end
        elseif alpha>112.5 && alpha<=157.5 || alpha<-22.5 && alpha>-65.5
            %diagonal obt
            if mag(row,col)>=mag(row-1,col-1) &&  mag(row,col)>=mag(row+1,col+1)
                %max
                out(row,col)=mag(row,col);
            end
        end  
    end
end
    

figure('name','aftermaxsup')
out=uint8(out)
imshow(out)

%% Apply Hysteresis threshold:

low_threshold=20
high_threshold=100

out(out<low_threshold)=0;
out(out>high_threshold)=255;

figure('name','afterhighlow')
imshow(out)
weak_indices=out>=low_threshold & out<=high_threshold;
% general
%TOP LEFT TO BOTTOM right
%bootom right to top left
%Top right to bottom left
%bottom left to top right
%row col
startpt=[1,1;
    size(I,1),size(I,2);
    1,size(I,2);
    size(I,1),1]
endpt=[size(I,1),size(I,2);
            1,1;
            size(I,1),1;
            1,size(I,1)
            ]
direction=[1,1;
    -1,-1;
    1,-1;
    -1,1]
for i=1:2 % looping through diff start end
    for row=startpt(i,1):direction(i,1):endpt(i,1)
        for col=startpt(i,2):direction(i,2):endpt(i,2)

%Loop for every pixel

%check if it is weak
value=out(row,col);
if value<=high_threshold && value>=low_threshold
    %if weak check its neighbours
    if out(row-1,col-1)>=high_threshold || out(row-1,col)>=high_threshold || out(row-1,col+1)>=high_threshold || out(row,col-1)>=high_threshold  || out(row,col+1)>=high_threshold || out(row+1,col-1)>=high_threshold || out(row+1,col)>=high_threshold || out(row+1,col+1)>=high_threshold
        %if one of its neighbours is max set it to 255
        out(row,col)=255;    
    end
end
        end
    end
end

out(out~=255)=0
figure('name','afterhysteresis')
imshow(out)


%%compare to matlab
BW1 = edge(I,'Canny' );
figure('name','matlabcanny')
imshow(BW1)



