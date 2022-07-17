
%% reading images

for n=1:7
  Cone{n} = imread(sprintf('Cone%d.png',n));
  Cone{n} = rgb2gray(Cone{n});

%   To obtain accurate albedo results, we must
%   normalize the values of the image intensity pixel values which is the
%   purpose of the commented step below. However, we opted to keep the
%   MATLAB range for each pixel from 0-255 which gave much better accuracy.

%   Cone{n} = Cone{n}/256; 
end

for n=1:7
  Icosphere{n} = imread(sprintf('Icosphere%d.png',n));
  Icosphere{n} = rgb2gray(Icosphere{n});
%   Icosphere{n} = Icosphere{n}/256;
end

for n=1:7
  Monkey{n} = imread(sprintf('Monkey%d.png',n));
  Monkey{n} = rgb2gray(Monkey{n});
%   Monkey{n} = Monkey{n}/256;
end

for n=1:7
  Torus{n} = imread(sprintf('Torus%d.png',n));
  Torus{n} = rgb2gray(Torus{n});
%   Torus{n} = Torus{n}/256;
end

%%  Light Source vectors

lightpos = [7 -1 5;
            -9 -2 6;
            -4 5 7;
            2 3 10;
            10 0 10
            0 -7 7;
            0 0 10];

objectpos = [0 0 0]; % ceter of the object

    S = zeros(7,3); % source vector in each row

for i=1:7
    S(i,:) =  lightpos(i,:) - objectpos ; % vector from center of object towards each ligth source 
    S(i,:) = S(i,:)/norm(S(i,:)); % normalize vector 
    %we should multiply it with the mag of ligh source but assume it is one
end
    k=1;
    V = k*S; % assuming k=1 
    % V will be same for all pixels due to the fact of Si is same (approximate by taking the lightpose - center rather than pixel loc)
    %pseudoV= (transpose(V)*V)\transpose(V); % pseudo inverse of V
    pseudoV= inv(transpose(V)*V)*transpose(V); % pseudo inverse of V same as above
%% obtaining pixel intensity vector and inferring normal for each point in each image

for j=1:4
    i = zeros(256,256,7); 
    g = zeros(256,256,3);
    N = zeros(256,256,3);
    ivec = zeros(7,1);
    gvec = zeros(3,1);
    p=zeros(256,256);
    q=zeros(256,256);
    hm = zeros(256,256);

    if j ==1
        %for each pixel
        for x = 1:256
            for y = 1:256
                %stack the pixel intensity for all the images/ilumination 
                i(y,x,:) = [Cone{1}(y,x);Cone{2}(y,x);Cone{3}(y,x);Cone{4}(y,x);Cone{5}(y,x);Cone{6}(y,x);Cone{7}(y,x)];
                ivec(:) = i(y,x,:);
                
                g(y,x,:)=pseudoV*ivec; % solve for g for this pixel location
                
                gvec(:) = g(y,x,:); % find albedo
                
                %find normal
                if norm(gvec) == 0
                    N(y,x,:) = [0,0,1];
                else
                N(y,x,:)=gvec/norm(gvec);
                end
                
                % find q and p ()
                p(y,x) = N(y,x,1)/N(y,x,3);
                q(y,x) = N(y,x,2)/N(y,x,3);

            end
        end
        figure(1);
        set(1,'name','Cone')
    end

    if j ==2
         for x = 1:256
            for y = 1:256
                i(y,x,:) = [Icosphere{1}(y,x);Icosphere{2}(y,x);Icosphere{3}(y,x);Icosphere{4}(y,x);Icosphere{5}(y,x);Icosphere{6}(y,x);Icosphere{7}(y,x)];
                ivec(:) = i(y,x,:);
                g(y,x,:)=pseudoV*ivec;
                gvec(:) = g(y,x,:);
                if norm(gvec) == 0
                    N(y,x,:) = [0,0,1];
                else
                N(y,x,:)=gvec/norm(gvec);
                end
                p(y,x) = N(y,x,1)/N(y,x,3);
                q(y,x) = N(y,x,2)/N(y,x,3);
                
            end
         end
        figure(2);
        set(2,'name','Icosphere')
    end
    
    if j ==3
        for x = 1:256
            for y = 1:256
                i(y,x,:) = [Monkey{1}(y,x);Monkey{2}(y,x);Monkey{3}(y,x);Monkey{4}(y,x);Monkey{5}(y,x);Monkey{6}(y,x);Monkey{7}(y,x)];
                ivec(:) = i(y,x,:);
                g(y,x,:)=pseudoV*ivec;
                gvec(:) = g(y,x,:);
                if norm(gvec) == 0
                    N(y,x,:) = [0,0,1];
                else
                N(y,x,:)=gvec/norm(gvec);
                end
                p(y,x) = N(y,x,1)/N(y,x,3);
                q(y,x) = N(y,x,2)/N(y,x,3);
            end
        end
        figure(3);
        set(3,'name','Monkey')
    end

    if j ==4
        for x = 1:256
            for y = 1:256
                i(y,x,:) = [Torus{1}(y,x);Torus{2}(y,x);Torus{3}(y,x);Torus{4}(y,x);Torus{5}(y,x);Torus{6}(y,x);Torus{7}(y,x)];
                ivec(:) = i(y,x,:);
                g(y,x,:)=pseudoV*ivec;
                gvec(:) = g(y,x,:);
                if norm(gvec) == 0
                    N(y,x,:) = [0,0,1];
                else
                N(y,x,:)=gvec/norm(gvec);
                end
                p(y,x) = N(y,x,1)/N(y,x,3);
                q(y,x) = N(y,x,2)/N(y,x,3);
            end
        end
        figure(4);
        set(4,'name','Torus')
    end
    
%% Integration: building 3D image 

    for i = 2:256

        hm(i,1) = hm(i-1,1) + q(i,1);

    end

    for i = 1:256
        
        for k = 2:256
            hm(i,k) = hm(i,k-1) + p(i,k);
        end

    end

%%  method 2 for problem 2.15 part d

%     for i = 2:256
% 
%         hm(i,1) = hm(i-1,1) + q(i,1);
% 
%     end
% 
%     for i = 2:256
% 
%         hm(1,i) = hm(1,i-1) + p(1,i);
% 
%     end
% 
%     for i = 2:256
%         
%         for k = 2:256
%             hm1 = hm(i,k-1) + p(i,k);
%             hm2 = hm(i-1,k) + q(i,k);
%             hm(i,k) = (hm1+hm2)/2;
%         end
% 
%     end
%%  Plotting height map

    surf(-hm);
    if j==1
        title('Cone');
    end
    if j==2
        title('Icosphere');
    end
    if j==3
        title('Monkey');
    end
    if j==4
        title('Torus');
    end

end