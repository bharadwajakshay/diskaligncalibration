function projectedPoints = project3DpointonPlane(points,model)
    debug =false;
    %% This works 
    dist = dot(repmat(model.Normal,[size(points,1),1]),points,2) + model.Parameters(4);
    projectedPoints = points - (dist * model.Normal);
    if debug
        figure;
        pcshow(pointCloud(projectedPoints))
    end
    

    %% SVD based
    %meanPt = mean(points,1);

    %meanSubPoints = points - meanPt;

    %[U,S,V] = svd(meanSubPoints);
    %n = V(:,3);
end