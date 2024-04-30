function [resortedCenters] = organizeToGrid(centers,dist)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

[val,idx] = sort(centers(:,2));
%sortedCenters = zeros(size(centers));
sortedCenters = centers(idx,:);
cluster = dbscan(sortedCenters(:,2),[dist],1);

noOfClusters = max(cluster);

resortedCenters = zeros(size(centers));

for i = 1:noOfClusters
    clusterPts = sortedCenters(cluster==i,:);
    [val,idx] = sort(clusterPts(:,1));
    resortedCenters(cluster==i,:) =  clusterPts(idx,:);
end

end