clc
clear all

%{  

zhang method

we still need to get at least 3 pictures, and get at least 5 correspondance
ponts for each

%}
%% get correspondance
%% Read Image

for i = 1:20
      imageFileNames = sprintf('%d.jpeg', i);
  
      I{i}= imread(imageFileNames);
      I{i}=rgb2gray(I{i}); % convert to grayscale
     
 end


%% Finding points 
for i=1:20
[imagePoints{i}, boardSize] = detectCheckerboardPoints(I{i});

J {i}= insertText(I{i},imagePoints{i},1:size(imagePoints{i},1));
J{i} = insertMarker(J{i},imagePoints{i},'o','Color','red','Size',5);
figure;
imshow(J{i});
end

%% World Frame coordinates for all points
k=1;
for x=1:boardSize(2)-1
    for y=1:boardSize(1)-1
       
        WorldFrame(k,1)=x-1;
        WorldFrame(k,2)=y-1;
        k=k+1;
    end
end


%% fill points arrays
%assume we got the points for each image
%(elements,points,images)
%Points_3d=ones(2,5,3)  % two rows since z is always zero
N_imgs=size(imagePoints,2)


%fill the 2D points
for i=1:N_imgs
    
if i==1    
    Points_2d=imagePoints{1}'

else
   Points_2d=cat(3,Points_2d,imagePoints{i}') 
end
end

%fill the 3d points
for j=1:N_imgs
Points_3d(:,:,j)=WorldFrame'
end

N_pts=size(Points_3d,2)
%N_imgs=size(Points_3d,3)
P_M_imgs=zeros(2*N_pts,9,N_imgs)

%% fill P_M for each image
for d=1:N_imgs
    for n=1:N_pts

        P_m=[ 
              Points_3d(:,n,d)' 1 zeros(1,3) -Points_2d(1,n,d)*Points_3d(:,n,d)' -Points_2d(1,n,d)*1 ;

              zeros(1,3) Points_3d(:,n,d)' 1 -Points_2d(2,n,d)*Points_3d(:,n,d)' -Points_2d(2,n,d)*1

             ]

         if n==1
             P_M=P_m;
         else 
             P_M=cat(1,P_M,P_m);
         end
         
        
    end
    
     P_M_imgs(:,:,d)=P_M
    
end

%% solve for each image

for d=1:N_imgs
    
    %get Ptranspose*P
    U=P_M_imgs(:,:,d)'*P_M_imgs(:,:,d);

    %get the eigen decomposition
    [V,D,W] = eig(U);
    %get min eigen value with the corresponding eigen vector
    evector_min=V(:,1); 
    %solution
    h=evector_min;
    %fill H
    H(:,:,d)=[ h(1:3)'; h(4:6)' ; h(7:9)' ]

end

%% fill V where Vb=0

for d=1:N_imgs
    
    h=H(:,:,d)
    
    i=1
    j=2
    v12=[ h(i,1)*h(j,1), h(i,1)*h(j,2)+h(i,2)*h(j,1), h(i,2)*h(j,2), h(i,3)*h(j,1)+h(i,1)*h(j,3), h(i,3)*h(j,2)+h(i,2)*h(j,3), h(i,3)*h(j,3)  ]
    
    i=1
    j=1
    v11=[ h(i,1)*h(j,1), h(i,1)*h(j,2)+h(i,2)*h(j,1), h(i,2)*h(j,2), h(i,3)*h(j,1)+h(i,1)*h(j,3), h(i,3)*h(j,2)+h(i,2)*h(j,3), h(i,3)*h(j,3)  ]

    i=2
    j=2
    v22=[ h(i,1)*h(j,1), h(i,1)*h(j,2)+h(i,2)*h(j,1), h(i,2)*h(j,2), h(i,3)*h(j,1)+h(i,1)*h(j,3), h(i,3)*h(j,2)+h(i,2)*h(j,3), h(i,3)*h(j,3)  ]
    
    % V for one image
    V_img=[ v12 ; (v11-v22) ]
    
    if d==1
        V_full=V_img
        
    else 
        V_full=cat(1,V_full,V_img)
    end


end

%% solve V_full.b=0

%get Ptranspose*P
U=V_full'*V_full;

%get the eigen decomposition
[V,D,W] = eig(U);
%get min eigen value with the corresponding eigen vector
evector_min=V(:,1); 
%solution
b=evector_min;
%fill M
B=[b(1),b(2),b(4); b(2),b(3),b(5); b(4),b(5),b(6)]

%% find the intrinsics

v0=( B(1,2)*B(1,3) - B(1,1)*B(2,3) ) /( B(1,1)*B(2,2)-B(1,2)^2  ) 

lamda= B(3,3) - ( B(1,3)^2 + v0*( B(1,2)*B(1,3) - B(1,1)*B(2,3) ) )  / B(1,1)

alpha=( lamda/B(1,1) )^1/2

beta= ( lamda*B(1,1) / ( B(1,1)*B(2,2) - B(1,2)^2 ) )^1/2

gamma=-B(1,2)*(alpha^2) *beta / lamda

u0=gamma*v0 /( beta - ( B(1,3)*alpha^2 )/lamda )

%% fill A (not sure that gamma =  the secon element in A)###
theta=acot(-gamma/alpha)

A=[alpha,     gamma     ,  u0 ; 
    0,   beta/sin(theta) , v0 ;
    0,       0           ,  1]

%% get extrinsics for each camera location

for d=1:N_imgs
    
    
r1=lamda*inv(A)*H(1,:,d)'
r2=lamda*inv(A)*H(2,:,d)'
r3=cross(r1,r2)

t=lamda*inv(A)*H(3,:,d)'

%transformation image for each camera
T(:,:,d)=[ r1' t(1); r2' t(2) ;r3' t(3)] 


end




%% just for testing
%choose point
total_error_px=0
total_error_py=0
for image=1:N_imgs
    for n=1:N_pts 
    %reprojection 
    trans=H(:,:,image)
    p_2d=trans*[Points_3d(:,n,image);1]%note that i am adding one for homo 
    %change from euclidian to homo
    p_2d=p_2d/p_2d(3)
    %true value
    Points_2d(:,n,image)
    error=abs(Points_2d(:,n,image)-p_2d(1:2))

    total_error_px=total_error_px + error(1)
    total_error_py=total_error_py + error(2)
    end

end
avg_error_px = total_error_px / (N_pts*N_imgs)
avg_error_py = total_error_py / (N_pts*N_imgs)



