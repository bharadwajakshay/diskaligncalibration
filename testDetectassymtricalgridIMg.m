clear;
clc;
close all; 
myDir = "../data/lounge";
%myDir = "data/Basement Data(New Board)"; %gets directory
%myDir = "data/Outside Data(New Board)"; %gets directory
%myFiles = dir(fullfile(myDir,'*.jpeg'));
myFiles = dir(fullfile(myDir,'*.jpg'));

debug = false;

counter = 0;
valid = 0;

intrinsics = load(['/Users/akshay/Documents/programming/' ...
    'camera-LiDAR_Calibration/recalibration/front_left_Cam_intrinsics.mat'] ...
    ).cameraParams.Intrinsics;

extrinsics = load(['/Users/akshay/Documents/programming/' ...
    'camera-LiDAR_Calibration/recalibration/results_pre-calibration_board.mat']).tform;

imageCenters = [];
pointCldCenters = [];

for k =1:length(myFiles)
     
    filename = fullfile(myFiles(k).folder,myFiles(k).name);
    strs = split(myFiles(k).name,'.');
    ptCldFileName = fullfile(myFiles(k).folder, join([strs(1),'pcd'],'.'));

    fprintf("Image: %s \nPoint Cloud: %s\n\n",filename,ptCldFileName{1});
     


     img_orig = imread(filename);
     undistedImg = undistortImage(img_orig,intrinsics);
     if debug
         figure;
        subplot(1,2,1);
        imshow(img_orig);
        subplot(1,2,2);
        imshow(undistedImg);
     end
     

     centers = detectCalibBoard(undistedImg);
     if ~isempty(centers)
        ptcld = pcread(ptCldFileName{1});
        ptcldCenters = detectCalibBoardLiDAR(ptcld, intrinsics, extrinsics,img_orig);
        
        if length(ptcldCenters) == length(centers)
            imageCenters = [imageCenters;centers];
            pointCldCenters = [pointCldCenters;ptcldCenters];
            valid = valid +1;
        end
        
     end
     counter = counter + 1;
     pause(1);

     % Processing Point Cloud
     
end

