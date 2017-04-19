function visualize_cameras(cams)
%
% visualize_cameras(cams)
%    
% 3d visualization of camera poses as generated
% by the extrinsic calibration tool
%
% cams:  n x 7 array with camera poses, as produced by calibration tool
%         
%  1: id  2:4 rotation_vector 5:7 translation vector
%
% The transform is from camera coordinates to world coordinates
% 
%
% example usage:
%
% visualize_cameras(textread('../test/cam_poses.txt'));
% 
%
    len = 0.1;
    ih = ishold;
    for i=1:size(cams,1)
        name = sprintf('cam %d', cams(i,1));
        rvec = [1 0 0 0];
        r    = cams(i,2:4);
        if (norm(r) > 1e-7)
            rvec = [r/norm(r), norm(r)];
        end
        T = [vrrotvec2mat(rvec),cams(i,5:7)';[0 0 0 1]];
        visualize_coordinate_system(T, name, 0.1);
        hold on;
    end
    if ih 
        hold on
    else
        hold off
    end
    axis equal;
end
