clear all
clc
%{
we will use the direct linear transform
%}

%% correspomdance points
% knowing that each square is 10mm we can determine the correspoding point
% matrix in world coordinates

P = [10 4 0;
     10 2 0;
     9 0 3;
     11 0 1;
     10 0 2;
     8 2 0;
     0 4 10;
     9 2 0;
     10 0 4;
     10 1 0;
     9 3 0;
     9 9 0;
     1 9 0;
     0 8 4;
     8 0 4;
     11 1 0;
     1 0 10;
     0 9 3;
     0 1 10;
     0 8 10;
     9 0 5;
     0 2 11;
     7 0 5;
     1 7 0;
     2 0 9;
     1 3 0;
     10 1 0;
     8 6 0;
     9 2 0;
     9 4 0;
     4 8 0;
     2 0 10;
     7 3 0;
     3 7 0;
     0 10 4;
     0 9 9;
     11 0 3;
     4 0 8;
     9 7 0;
     9 1 0]*10;
 %The corner point locations in case the function varied or mixed the
 %results in later iterations
 p =[1.0071    0.8640;
    0.9923    0.9436;
    0.8495    1.0799;
    1.0002    1.0692;
    0.9255    1.0760;
    0.8987    0.8961;
    0.2321    0.9083;
    0.9455    0.9210;
    0.8622    1.1431;
    0.9865    0.9819;
    0.9502    0.8812;
    0.9927    0.6036;
    0.6246    0.4754;
    0.4466    0.5852;
    0.7739    1.0827;
    1.0342    1.0084;
    0.3059    1.0742;
    0.4799    0.5158;
    0.2695    1.0166;
    0.1632    0.7336;
    0.7829    1.1471;
    0.2108    1.0129;
    0.6992    1.0851;
    0.6309    0.5689;
    0.3785    1.0710;
    0.6412    0.7235;
    0.9854    0.9798;
    0.9198    0.7301;
    0.9444    0.9194;
    0.9578    0.8402;
    0.7455    0.5691;
    0.3380    1.1020;
    0.8605    0.8348;
    0.7071    0.5992;
    0.4293    0.4841;
    0.2002    0.6535;
    0.9412    1.1389;
    0.4847    1.0953;
    0.9780    0.7088;
    0.9398    0.9548]*1000;


Points_3d=P' %points in 3d
Points_2d=p' %points in 2d
N_pts=size(Points_3d,2) %number of pts

%% finding the transformation matrix
%P_m sub matrix of P_M full matrix

for n=1:N_pts
    
    P_m=[ 
          Points_3d(:,n)' 1 zeros(1,4) -Points_2d(1,n)*Points_3d(:,n)' -Points_2d(1,n)*1 ;

          zeros(1,4) Points_3d(:,n)' 1 -Points_2d(2,n)*Points_3d(:,n)' -Points_2d(2,n)*1
          
         ]
     
     if n==1
         P_M=P_m;
     else 
         P_M=cat(1,P_M,P_m);
     end
             
end


U=transpose(P_M)*P_M; %get Ptranspose*P
[V,D] = eig(U); %get the eigen decomposition
evector_min=V(:,1); %get min eigen value with the corresponding eigen vector
m=evector_min; %solution
M=[ m(1:4)'; m(5:8)' ; m(9:12)' ] %fill M


%% get intrinsics and exrinsics
%% fill A 

a1=M(1,1:3);
a2=M(2,1:3);
a3=M(3,1:3);

A=[a1;a2;a3];

%% get the intrinsics and extrinsics from A

epsilon=1 % 1 or -1
raw=epsilon/norm(a3) %get raw
b=M(:,4)/raw; %get b
r3=raw*a3 %get row 3


x0=raw^2*(a1*a3') % get x0 
y0=raw^2*(a2*a3') % get y0

% get theta
c1=cross(a1,a3) % dumy variable 
c2=cross(a2,a3) % dumy variable
dumy=-( (  c1*c2' ) / ( norm(c1)*norm(c2) ) )
theta= acos(dumy) %output angle in radians

%get alpha and beta
alpha=(raw^2)*norm(c1)*sin(theta)
beta=(raw^2)*norm(c2)*sin(theta)

%fill K matrix
K=[alpha, -alpha*cot(theta), x0;
    0   ,  beta/sin(theta)  ,  y0;
    0   ,      0            ,   1   
    ]

%get the translational vector

t=raw*inv(K)*b



%% just for testing
%choose point
total_error_px=0
total_error_py=0

for n=1:N_pts

%reprojection 
%find 1/Z * P(world pose) at the point n
p_2d=M*[Points_3d(:,n);1];
p_2d=p_2d/(M(3,:)*[Points_3d(:,n);1]);
p_2d=p_2d(1:2)

%true value
Points_2d(:,n)

error=abs(Points_2d(:,n)-p_2d(1:2))

total_error_px=total_error_px + error(1)
total_error_py=total_error_py + error(2)

end

avg_error_px = total_error_px / (N_pts)
avg_error_py = total_error_py / (N_pts)





 

