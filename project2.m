% Computer vision project, 3D reconstruction from stereo vision using a
% kinect RGB and depth sensor
% Writen by Edward Beeching & Usama Javaid

% The process is as follows: 
% 1. Set the intrinsic parameter for the left kinect RGB, left kinect RGB
% and the kinect depth sensor (we only know one set of parameters

% 2. Set the extrinsic parameters, right to left kinect and depth to RBG

% 3. Load the RBG and depth data from the PNG files and visualise them

% 4. For the Depth data, project into the 3D camera frame, visualise.

% 5. apply the extrinsic to transform to RGB camera frame, then transform 
% back to pixel coordinates.

% 6. The pixel image will have missing information, apply median filtering
% to the zeroes to fill in any gaps, median filtering is more suitable than
% mean when you have abrupt changes in depth

% 7. Project the RBG Pixel coordinates to 3D space, then for the right camera,
% use the extrinsic parameters to transform into the left camera
% coordinates


%% 1. Intrinsic parameters

% Left RGB sensor

fc_left_x = 0.529;
fc_left_y = 0.525;
cc_left_x = 0.296;
cc_left_y = 0.242;

left_distortion = [ 0.24064   -0.75950   -0.01266   -0.01777  0.00000 ]; % I ignore distortion for the moment
% The left M3 matrix for pixel to metric
left_M3 =    [320/cc_left_x     0               320;  
              0                 240/cc_left_y   240;
              0                 0               1];
left_M3inv = inv(left_M3);

% Right RGB sensor
fc_right_x = 0.542;
fc_right_y = 0.533;
cc_right_x = 0.265;
cc_right_y = 0.254;

right_distortion = [ 0.24064   -0.75950   -0.01266   -0.01777  0.00000 ]; % I ignore distortion for the moment

right_M3 =    [320/cc_right_x    0           320;
              0             240/cc_right_y   240;
              0             0           1];
right_M3inv = inv(right_M3);


% Intrinsic paramters for Depth Sensor
% Kinect Depth camera parameters
fx_d =  5.7616540758591043e-01;
fy_d = 5.7375619782082447e-01;
cx_d = 3.2442516903961865e-01;
cy_d = 2.3584766381177013e-01;



depth_M3 = [320/cx_d    0           320;
              0         240/cy_d    240;
              0         0           1];
          
depth_M3_inv = inv(depth_M3);


%% 2. Set the Extrinsic Parameters
% Extrinsic parameters between RGB and Depth camera for Kinect V1
% Rotation matrix
R_depth =  inv([  9.9998579449446667e-01, 3.4203777687649762e-03, -4.0880099301915437e-03;
    -3.4291385577729263e-03, 9.9999183503355726e-01, -2.1379604698021303e-03;
    4.0806639192662465e-03, 2.1519484514690057e-03,  9.9998935859330040e-01]);

% Translation vector.
T_depth = -[  2.2142187053089738e-02, -1.4391632009665779e-04, -7.9356552371601212e-03 ]';

% Extrinsic between from right to left camera

% Rotation vector:             om = [ -0.03582   0.74776  -0.03287 ] ± [ 0.02024   0.04000  0.00849 ]
% Translation vector:           T = [ -810.93822   22.48708  397.39319 ] ± [ 15.35103   7.27743  36.49780 ]



T_right_to_left = [0.8109 0.022 0.397];


%% 3. Load the RBG and depth data from the PNG files and visualise them

left_image = imread('LeftRgbobjects11.png');
left_depth = imread('LeftDepthobjects11.png');

right_image = imread('RightRgbobjects11.png');
right_depth = imread('RightDepthobjects11.png');

% Scale the Z coordinates so they are in cm
left_depth_adjusted =  double(left_depth)/1000.0;
right_depth_adjusted =  double(right_depth)/1000.0;

% Lets have a look at the images
figure();
imshow(left_image);
figure();
imshow(left_depth_adjusted,[0.8,3.0],'Colormap', jet(255));
figure();
imshow(right_image);
figure();
imshow(right_depth_adjusted,[0.8,3.0],'Colormap', jet(255));


%% 4. For the Depth data, project into the 3D camera frame, visualise, apply the
% extrinsic to transform to RGB frame, then transform back to pixel
% coordinates.
[height, width] = size(left_depth_adjusted);
% First generate an array of X,Y,Z coordinates

% For the left depth sensor
left_depth_dccf = zeros(3,height*width); % dcc = depth camera coordinate frame

for i = 1:height
    for j = 1:width
        image_coords = depth_M3_inv  * [j i 1]';
        left_depth_dccf(1,(i-1)*width + j) = image_coords(1) * left_depth_adjusted(i,j)/fx_d;
        left_depth_dccf(2,(i-1)*width + j) = image_coords(2) * left_depth_adjusted(i,j)/fy_d;
        left_depth_dccf(3,(i-1)*width + j) = left_depth_adjusted(i,j);
    end
end

% For the right depth sensor
right_depth_dccf = zeros(3,height*width); % dcc = depth camera coordinate frame

for i = 1:height
    for j = 1:width
        image_coords = depth_M3_inv  * [j i 1]';
        right_depth_dccf(1,(i-1)*width + j) = image_coords(1) * right_depth_adjusted(i,j)/fx_d;
        right_depth_dccf(2,(i-1)*width + j) = image_coords(2) * right_depth_adjusted(i,j)/fy_d;
        right_depth_dccf(3,(i-1)*width + j) = right_depth_adjusted(i,j);
    end
end

% visualise with point cloud show to check this looks ok.
pcshow(left_depth_dccf')
figure()
pcshow(right_depth_dccf')


%% 5. Used the tranlation and rotation matrix to move the XYZ depth data to
% camera coordinate frame

% Transform the left XYZ
left_depth_rbgccf = zeros(3,height*width); % rgbcc = RGB camera coordinate frame

left_depth_rbgccf = R_depth* left_depth_dccf;
left_depth_rbgccf = bsxfun(@plus,left_depth_rbgccf,T_depth); % nice helper function for applying a tranlation to matrix of vectors


% Transform the right XYZ
right_depth_rbgccf = zeros(3,height*width); % rgbcc = RGB camera coordinate frame

right_depth_rbgccf = R_depth* right_depth_dccf;
right_depth_rbgccf = bsxfun(@plus,right_depth_rbgccf,T_depth); % nice helper function for applying a tranlation to matrix of vectors


%left_depth_rbgccf =left_depth_dccf;
%right_depth_rbgccf =right_depth_dccf;

% Generate two new images for the adjusted depth per pixel in the camera
% coordinate system

% Left camera
left_depth_map = zeros(height,width);
for i = 1:height
    for j = 1:width
        % I make sure here to use the focal lengths of the left RBG camera
        x = left_depth_rbgccf(1,(i-1)*width + j) * (fc_left_x /  left_depth_rbgccf(3,(i-1)*width + j)); 
        y = left_depth_rbgccf(2,(i-1)*width + j) * (fc_left_y /  left_depth_rbgccf(3,(i-1)*width + j));
        
        % and here I use the M3 matrix for the RGB camera
        pixel_coords = left_M3 * [x y 1]';
        
        % Round to pixel coordinates
        u = round(pixel_coords(1));
        v = round(pixel_coords(2));
        
        % Make sure we are within the height and width of the depth map
        if(u > 0 && u <= width && v > 0 && v <= height)
            left_depth_map(v,u) =  left_depth_rbgccf(3,(i-1)*width + j);
        end
        
    end
end

% Right Camera

right_depth_map = zeros(height,width);
for i = 1:height
    for j = 1:width
        % I make sure here to use the focal lengths of the left RBG camera
        x = right_depth_rbgccf(1,(i-1)*width + j) * (fc_right_x /  right_depth_rbgccf(3,(i-1)*width + j)); 
        y = right_depth_rbgccf(2,(i-1)*width + j) * (fc_right_y /  right_depth_rbgccf(3,(i-1)*width + j));
        
        % and here I use the M3 matrix for the RGB camera
        pixel_coords = right_M3 * [x y 1]';
        
        % Round to pixel coordinates
        u = round(pixel_coords(1));
        v = round(pixel_coords(2));
        
        % Make sure we are within the height and width of the depth map
        if(u > 0 && u <= width && v > 0 && v <= height)
            right_depth_map(v,u) =  right_depth_rbgccf(3,(i-1)*width + j);
        end
        
    end
end

% Lets visualise the new depth maps

figure();
imshow(left_depth_map,[0.8,3.0],'Colormap', jet(255));
    
figure();
imshow(right_depth_map,[0.8,3.0],'Colormap', jet(255));
    
%% 6. Due to the transformation, the new depth maps contain many zero values
% The edges do not matter but it would be nice to fill in the gaps in the
% centre of the image

% A few passes of median filtering on the zero values should do a
% reasonable job here.
passes_of_median = 2;
% Median filtering on left image
left_depth_map_med = left_depth_map;
for i = 1:passes_of_median
    for i = 1:height
        for j = 1:width

           if(left_depth_map_med(i,j) < 0.1) 
                window = left_depth_map(max(i-5, 1):min(i+5, height),max(j-5, 1):min(j+5, width));
                left_depth_map_med(i,j) = median(median(window)); % I choose median here as mean would smooth over boundaries
           end


        end
    end
end

% Median filtering on left image
right_depth_map_med = right_depth_map;
for i = 1:passes_of_median
    for i = 1:height
        for j = 1:width

           if(right_depth_map_med(i,j) < 0.1) 
                window = right_depth_map_med(max(i-5, 1):min(i+5, height),max(j-5, 1):min(j+5, width));
                right_depth_map_med(i,j) = median(median(window)); % I choose median here as mean would smooth over boundaries
           end


        end
    end
end

% Visualise the before and after to see the improvement
figure();
imshow(left_depth_map,[0.8,3.0],'Colormap', jet(255));
    
figure();
imshow(right_depth_map,[0.8,3.0],'Colormap', jet(255));    
    
    
figure();
imshow(left_depth_map_med,[0.8,3.0],'Colormap', jet(255));
    
figure();
imshow(right_depth_map_med,[0.8,3.0],'Colormap', jet(255)); 
    
%% 7. Now we have calibrated depth maps lets transform the u,v RGB to XYZ RGB

% For the left camera
left_rbg_ccf = zeros(3,height*width);
left_rbg_colors = zeros(3,height*width,'uint8');
for i = 1:height
    for j = 1:width
        
        % I use an if here as I am not interested in coordinates that have
        % a low Z value
        if(left_depth_map_med(i,j) > 0.1)
            image_coords = left_M3inv  * [j i 1]';
            left_rbg_ccf(1,(i-1)*width + j) = image_coords(1) * left_depth_map_med(i,j)/fc_left_x;
            left_rbg_ccf(2,(i-1)*width + j) = image_coords(2) * left_depth_map_med(i,j)/fc_left_y;
            left_rbg_ccf(3,(i-1)*width + j) = left_depth_map_med(i,j);
            
            left_rbg_colors(1,(i-1)*width + j) = left_image(i,j,1);
            left_rbg_colors(2,(i-1)*width + j) = left_image(i,j,2);
            left_rbg_colors(3,(i-1)*width + j) = left_image(i,j,3);
        end
        
        
    end
end

% For the right camera
right_rbg_ccf = zeros(3,height*width);
right_rbg_colors = zeros(3,height*width,'uint8');
for i = 1:height
    for j = 1:width
        
        % I use an if here as I am not interested in coordinates that have
        % a low Z value
        if(right_depth_map_med(i,j) > 0.1)
            image_coords = right_M3inv  * [j i 1]';
            right_rbg_ccf(1,(i-1)*width + j) = image_coords(1) * right_depth_map_med(i,j)/fc_right_x;
            right_rbg_ccf(2,(i-1)*width + j) = image_coords(2) * right_depth_map_med(i,j)/fc_right_y;
            right_rbg_ccf(3,(i-1)*width + j) = right_depth_map_med(i,j);
            
            right_rbg_colors(1,(i-1)*width + j) = right_image(i,j,1);
            right_rbg_colors(2,(i-1)*width + j) = right_image(i,j,2);
            right_rbg_colors(3,(i-1)*width + j) = right_image(i,j,3);
        end
        
        
    end
end 

% lets visual these two point clouds
figure()
pcshow(left_depth_dccf',left_rbg_colors')
figure()
pcshow(right_depth_dccf',right_rbg_colors') 
    
    
    
%% Now to translate the right camera coordinate frame to the left one
thetax = -0.03582;
thetay = +0.74776;
thetaz = -0.03287;

% -0.03582   0.74776  -0.03287

rz = [cos(thetaz) -sin(thetaz)   0;
      sin(thetaz) cos(thetaz) 0;
      0             0             1]  ;

rx =  [1  0 0 ;
      0   cos(thetax) -sin(thetax);
      0   sin(thetax) cos(thetax)]; 



Ry_right_to_left  = [cos(thetay) 0 -sin(thetay);
                        0 1 0;
                        sin(thetay) 0 cos(thetay)];


T_right_to_left = [-0.8109 0.022 0.397];

right_to_left_rbg_ccf = bsxfun(@minus,right_rbg_ccf,T_right_to_left');
right_to_left_rbg_ccf = Ry_right_to_left * right_to_left_rbg_ccf;

figure()
pcshow(right_to_left_rbg_ccf',right_rbg_colors') 



figure()
pcshow([left_depth_dccf right_to_left_rbg_ccf]',[left_rbg_colors right_rbg_colors]')


