function write_tags(tags, filename);
%
% write_tags(tags, filename);
%
% 2017 Bernd Pfrommer
%    
% tags = n x 14 array, where n = number of tags (about 191)
%
% The columns have the following meaning:
% 1     2          3-5        6-8       9-11      12-14
% tagid tagsize[m] position,  rotation  posnoise  rotation noise
%
%    
% usage:
%
% write_tags(make_tags(), '../config/tag_poses.yaml');
    
    fid = fopen(filename, 'w');
    for i=1:size(tags, 1)
        fprintf(fid, '- id: %d\n', tags(i,1));
        fprintf(fid, '  size: %f\n', tags(i,2));
        fprintf(fid, '  center:\n');
        fprintf(fid, '     x: %f\n', tags(i,3));
        fprintf(fid, '     y: %f\n', tags(i,4));
        fprintf(fid, '     z: %f\n', tags(i,5));
        fprintf(fid, '  rotvec:\n');
        fprintf(fid, '     x: %f\n', tags(i,6));
        fprintf(fid, '     y: %f\n', tags(i,7));
        fprintf(fid, '     z: %f\n', tags(i,8));
        fprintf(fid, '  position_noise:\n');
        fprintf(fid, '     x: %f\n', tags(i,9));
        fprintf(fid, '     y: %f\n', tags(i,10));
        fprintf(fid, '     z: %f\n', tags(i,11));
        fprintf(fid, '  rotation_noise:\n');
        fprintf(fid, '     x: %f\n', tags(i,12));
        fprintf(fid, '     y: %f\n', tags(i,13));
        fprintf(fid, '     z: %f\n', tags(i,14));
    end
    fclose(fid);
    disp(sprintf('wrote %d tags', size(tags,1)));
end
