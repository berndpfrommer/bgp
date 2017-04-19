function plot_reprojection_data(a)
%
% plot_reprojection_data(a)
%
% plots reprojection data in the array a
%
% a = n x 13 array as produced by the calibration tool
%
% example usage:
%
% plot_reprojection_data(textread('../test/reproj.txt'));
%
    camidx = 2;
    cams = unique(a(:,camidx));
    for i=1:size(cams)
        camid = cams(i);
        figure;
        idx = find(a(:,camidx) == camid);
        plot(a(idx,8),a(idx,9), 'o', a(idx,10), a(idx,11),'.', a(idx,12), ...
             a(idx,13),'+');
        title(sprintf('camera id: %d', camid));
        legend('original', 'reprojected', 'reprojected optimized');
    end
end
