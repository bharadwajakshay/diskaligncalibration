function [resortedCenters, resortedBB] = organizeLiDARToGrid(centroids,BB,dist)

[val,idx] = sort(centroids(:,2));
%sortedCenters = zeros(size(centers));
sortedCenters = centroids(idx,:);
sortedBB = BB(idx,:);
exit = false;
while(~exit)
    cluster = dbscan(sortedCenters(:,2),[dist],1);
    
    if dist > 0
        if (max(cluster) ~= 4) &  (max(cluster) ~= 10)
            dist=dist-1;
        else
            exit = true;
        end
    else
        resortedCenters= [];
        resortedBB = [];
        return
    end
end

noOfClusters = max(cluster);

resortedCenters = zeros(size(centroids));
resortedBB = zeros(size(sortedBB));

for i = 1:noOfClusters
    clusterPts = sortedCenters(cluster==i,:);
    culsterBB = sortedBB(cluster==i,:);
    [val,idx] = sort(clusterPts(:,1));
    resortedCenters(cluster==i,:) =  clusterPts(idx,:);
    resortedBB(cluster==i,:) = culsterBB(idx,:);

end

end
