function [centroids3D] = detectCalibBoardLiDAR(ptcld, intrinsics, extrinsics, img)

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
    
    filterdPtCld = filterPointCld(orgPtCld, idx, [H,W,C]);

    if debug
        figure;
        pcshow(filterdPtCld);

    end
end

intensityimg = uint8(filterdPtCld.Color);
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

%% Remove unwanted points
% remove ground point

[model,inlierIdices,outlierIndicies] = pcfitplane(filterdPtCld, 0.15,[0,0,1]);

grndRemovedPtCld = filterPointCld(filterdPtCld,outlierIndicies,[H,W,C]);

% get pc plane fit
[model,inlierIdices,outlierIndicies] = pcfitplane(grndRemovedPtCld, 0.15);

calibBoardPtCld = filterPointCld(grndRemovedPtCld,inlierIdices,[H,W,C]);

calibBoardIntThresh = imbinarize(calibBoardPtCld.Color(:,:,1)',"global");
calibBoardIntThresh = imcomplement(calibBoardIntThresh);

stat = regionprops(calibBoardIntThresh,"Area","BoundingBox","Centroid","ConvexArea");
area = vertcat(stat.Area);
centroid = vertcat(stat.Centroid);
BB = vertcat(stat.BoundingBox);

[val,idx] = rmoutliers(area,'median');
area = area(~idx);
centroid = centroid(~idx,:);
BB = BB(~idx,:);



[orgCentroid, BB] = organizeLiDARToGrid(centroid, BB, 5);



if ~isempty(orgCentroid)

calibBoardPtCldIm = im2uint8(calibBoardIntThresh).*255;


for i = 1:size(orgCentroid,1)
    calibBoardPtCldIm = insertText(calibBoardPtCldIm, [orgCentroid(i,1),orgCentroid(i,2)], ...
                            int2str(i),"BoxOpacity",0,"FontColor","yellow");
    %imshow(calibBoardPtCldIm);
    %plot(orgCentroid(i,1),orgCentroid(i,2),'g+', 'MarkerSize', 5, 'LineWidth', 2);

end

figure;
imshow(calibBoardPtCldIm)
hold on;
plot(orgCentroid(:,1),orgCentroid(:,2),'g+', 'MarkerSize', 5, 'LineWidth', 2);
hold off;



centroids3D = ones(size(centroid,1),3);

calibBoardIntThresh = calibBoardIntThresh';

figure;
pcshow(filterdPtCld);
set(gca,'color',[0.6, 0.6, 0.6]);

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

    if debug
        figure;
        pcshow(disk)
    end

    projectedPoints = project3DpointonPlane(disk.Location,model);

    centroids3D(diskIdx,:) = mean(projectedPoints);

    hold on;
    drawCircleonPlane(0.1,centroids3D(diskIdx,:),model.Normal,'r')
    h = scatter3(centroids3D(diskIdx,1), centroids3D(diskIdx,2), centroids3D(diskIdx,3),'filled','MarkerFaceColor',[0 .75 .75]);
    h.SizeData = 100;
    text(centroids3D(diskIdx,1),centroids3D(diskIdx,2),centroids3D(diskIdx,3), int2str(diskIdx),"Color","white","FontSize",24);

end

else
    centroids3D = [];
end

end