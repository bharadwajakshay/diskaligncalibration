function [sphere] = createSphere(points, p1, p2)
    % create unit vector
    vec = p2-p1;
    unitVec = vec/norm(vec);
    d = sqrt(unitVec(2)^2+unitVec(3)^2);
    T = eye(4);
    T(1:3,4) = p1';
    
    sphere = points;

    Rx = eye(4);
    Rx(2:3,2:3) = [unitVec(3)/d, -unitVec(2)/d; unitVec(2)/d, unitVec(3)/d];

    Ry = eye(4);
    Ry(1,1) = d;
    Ry(1,3) = -unitVec(1);
    Ry(3,1) =  unitVec(1);
    Ry(3,3) = d;
    
    rotVec = 0.1:pi/32:pi/2;


    homogPts = ones([size(points,1),4]);
    homogPts(:,1:3)= points;
    Rz = eye(4);

    for rot = 1:size(rotVec,2)
        Rz(1:2,1:2) = [cos(rotVec(rot)) -sin(rotVec(rot)); ...
            sin(rotVec(rot)) cos(rotVec(rot))];
        
        rotPoints = T^-1*Rx^-1*Ry^-1*Rz*Ry*Rx*T*homogPts';
        sphere = vertcat(sphere,rotPoints(1:3,:)');
    end

end