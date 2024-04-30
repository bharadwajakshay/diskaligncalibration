function [centers] = detectCalibBoard(image)
    img = rgb2gray(image);
    img = histeq(img);
    
    img(img>50) = 225;
    
    %img = imrotate(img,90);
    
    
    img(img<25) = 0;
    %imagePoints = detectCircleGridPoints(img,[4 5], "PatternType","asymmetric",CircleColor="black")
    [centers,radii] = imfindcircles(img,[20 100],ObjectPolarity="dark", Sensitivity=0.95,Method="PhaseCode",EdgeThreshold=0.5);
    
    % remove unwanted circles as radius
    [rad,idx] = rmoutliers(radii,'median');
    radii = radii(~idx);
    centers = centers(~idx,:);
    
    if isempty(centers)
        centers = [];
        return
    end
    [clusteridx,corePts] = dbscan(centers,[100],3,'Distance','minkowski');
    [GC,GR] = groupcounts(clusteridx);
    [m, idx] = max(GC);
    search_term = GR(idx);
    
    centers = centers(clusteridx == search_term,:);
    radii = radii(clusteridx == search_term);
    
    if size(centers,1) ~= 20
        centers = [];
        return
    end
    
    
    imgDetected = insertShape(image,"circle",[centers(:,1),centers(:,2),radii(:)],LineWidth=3, Color="red");
    
    resortedCenters = organizeToGrid(centers,20);
    
    for i = 1:size(resortedCenters,1)
    imgDetected = insertText(imgDetected, [resortedCenters(i,1),resortedCenters(i,2)],int2str(i),"BoxOpacity",0,"FontColor","white");
    end
    
    figure;imshow(imgDetected)
    hold on;
    plot(centers(:,1),centers(:,2),'g+', 'MarkerSize', 15, 'LineWidth', 2);
    hold off;
end