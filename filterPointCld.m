function [ptcloud] = filterPointCld(ptCld,idx,shape)
    % 
    H = shape(1);
    W = shape(2);
    C = shape(3);

    points = ptCld.Location;
    color = ptCld.Color;
    if length(size(points)) == 3
        points = reshape(points,[ptCld.Count,3]);
        color = reshape(color,[ptCld.Count,3]);
    end

    ptFilterd = ones(size(points)).*nan;
    ptFilterd(idx,:) = points(idx,:);

    clrFiltered = zeros(size(color));
    clrFiltered(idx,:) = color(idx,:);

    ptcloud = pointCloud(reshape(ptFilterd,[H, W, C]));
    ptcloud.Color = reshape(clrFiltered./255,[H, W, C]);





end