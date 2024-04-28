function [center] = detectCentersFromDisk(points)
    % get euclidean norm of 1 vector to the rest
    ed = ones([size(points,1),size(points,1)]).*nan;
    for i = 1:size(points,1)
        ed(:,i) = sqrt((points(i,1)-points(:,1)).^2 + (points(i,2)-points(:,2)).^2 ...
            +(points(i,3)-points(:,3)).^2);
    end
    [V,I] = max(ed);
    [V2,I2] = max(V);
    ptIdx = [I2,I(I2)];

    p1 = points(ptIdx(1),:);
    p2 = points(ptIdx(2),:);
    
    midpoint = [(p1(1)+p2(1))/2 ,(p1(2)+p2(2))/2, (p1(3)+p2(3))/2];

    sphere = createSphere(points, p1, p2);
end
