function [centers] = detectCalibBoardLiDAR(ptcld, intrinsics, extrinsics, img)

debug = false;

H=2000;
W=64;
C=3;
points = ptcld.Location;
colors = ptcld.Color;
orgPts = reshape(points,[H,W,C]);

orgPtCld = pointCloud(orgPts);
orgPtCld.Color = reshape(colors,[H,W,C]);

if ~isempty(extrinsics)
    % Filter the point cloud and get points 
    transformedCld = pctransform(orgPtCld,extrinsics);
    [imgpts,idx] = projectLidarPointsOnImage(orgPtCld, ...
        intrinsics, extrinsics,"ImageSize",intrinsics.ImageSize);
    if debug
        hold on;
        img = insertMarker(img ,imgpts,'*','color','blue','size', 3);
        imshow(img);
    end
    
    
    points_extracted = ones(size(points)).*nan;
    points_extracted(idx,:) = points(idx,:);
    points_extracted = reshape(points_extracted,[H,W,C]);
    filterdPtCld = pointCloud(points_extracted);
    filtColor = zeros(size(points));
    filtColor(idx,:) = colors(idx,:);

    filterdPtCld.Color = orgPtCld.Color;
    if debug
        figure;
        pcshow(filterdPtCld);

    end
end

intensityimg = zeros(size(points),"uint8");
intensityimg(idx,:) = uint8(colors(idx,:));
intensityimg = reshape(intensityimg,[H,W,C]);
if debug
    figure;
    imshow(intensityimg(:,:,1)');
end
intensityimg = intensityimg(:,:,1)';
intensityImgTresh = imbinarize(intensityimg,graythresh(intensityimg));
if debug
    figure;
    imshow(intensityImgTresh);
end

% get pc plane fit
[model,inlierIdices,outlierIndicies] = pcfitplane(filterdPtCld, 0.15);

calibBoardPts = ones(size(points)).*nan;
calibBoardPts(inlierIdices,:) = points(inlierIdices,:);

calibBoardPtCldclr = zeros(size(points),"uint8");
calibBoardPtCldclr(inlierIdices,:) = uint8(colors(inlierIdices,:));

calibBoardPtCld = pointCloud(reshape(calibBoardPts,[H,W,C]));
calibBoardPtCld.Color = orgPtCld.Color;

calibBoardIntensities = reshape(calibBoardPtCldclr,[H,W,C]);
calibBoardIntThresh = imbinarize(calibBoardIntensities(:,:,1)',"global");
calibBoardIntThresh = imcomplement(calibBoardIntThresh);

stat = regionprops(calibBoardIntThresh,"Area","BoundingBox","Centroid","ConvexArea");
area = vertcat(stat.Area);
centroid = vertcat(stat.Centroid);
BB = vertcat(stat.BoundingBox);

[val,idx] = rmoutliers(area,'median');
area = area(~idx);
centroid = centroid(~idx,:);
BB = BB(~idx,:);

% Swap the coloumns
centroidSwapped = zeros(size(centroid));
centroidSwapped(:,1) = centroid(:,2);
centroidSwapped(:,2) = centroid(:,1);

% BBSwapped = zeros(size(BB));
% 
% BBSwapped(:,1) = BB(:,2);
% BBSwapped(:,2) = BB(:,1);
% BBSwapped(:,3) = BB(:,4);
% BBSwapped(:,4) = BB(:,3);



orgCentroid = organizeToGrid(centroidSwapped);

%swap back the coloumns
orgCentroidcopy = zeros(size(orgCentroid));
orgCentroidcopy(:,1) = orgCentroid(:,2);
orgCentroidcopy(:,2) = orgCentroid(:,1);

orgCentroid = orgCentroidcopy;


changeIdx = zeros(size(orgCentroid,1),1);

for idx = 1:size(orgCentroid,1)
    changeIdx(idx) = find(orgCentroid(:,1) == centroid(idx,1));
end

calibBoardPtCldIm = im2uint8(calibBoardIntThresh).*255;

for i = 1:size(orgCentroid,1)
    calibBoardPtCldIm = insertText(calibBoardPtCldIm, [orgCentroid(i,1),orgCentroid(i,2)],int2str(i),"BoxOpacity",0,"FontColor","white");
end

figure;
imshow(calibBoardPtCldIm)
hold on;
plot(orgCentroid(:,1),orgCentroid(:,2),'g+', 'MarkerSize', 5, 'LineWidth', 2);
hold off;

BB = BB(changeIdx,:);

calibBoardIntThresh = calibBoardIntThresh';

for diskIdx=1:size(centroid,1)
    % get pixels
    idxR = round(BB(diskIdx,1):BB(diskIdx,1)+BB(diskIdx,3));
    idxC = round(BB(diskIdx,2):BB(diskIdx,2)+BB(diskIdx,4));

    % find only the valid points of the 
    [row,col] = find(calibBoardIntThresh(idxR,idxC));

    binaryMask = calibBoardIntThresh(idxR,idxC);
    binaryMask = repmat(binaryMask,[1,1,3]);
    
    newGridPts = calibBoardPtCld.Location(idxR,idxC,:);
    newGridPtsfilt = newGridPts.*binaryMask;
    newGridPtsfilt(newGridPtsfilt==0) = nan;


    diskPoints = pointCloud(newGridPtsfilt);

    [model, inlierIdices,outlierIndicies] = pcfitplane(diskPoints, 0.03);

    disk = select(diskPoints,inlierIdices);

    figure;
    pcshow(disk)

    projectedPoints = project3DpointonPlane(disk.Location,model);

    centroid = mean(projectedPoints);

    hold on;
    drawCircleonPlane(0.1,centroid,model.Normal,'r')
    h = scatter3(centroid(1), centroid(2), centroid(3),'filled','MarkerFaceColor',[0 .75 .75]);
    h.SizeData = 100;


    %center = detectCentersFromDisk(projectedPoints);

end

end