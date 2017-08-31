function write_masks(masks, filename);
%
% write_masks(masks, filename);
%
% 2017 Bernd Pfrommer
%    
% masks = container as created by make_cage
%
% usage:
%
% [tags, masks] = make_cage_2();
% write_masks(masks, 'masks.yaml');
%
    fid = fopen(filename, 'w');
    k = keys(masks);
    val = values(masks);
    for i = 1:length(masks)
        fprintf(fid, '%s:\n', k{i});
        fprintf(fid, '  masked_ids: [');
        v = val{i};
        for j = 1:(length(v) - 1)
            fprintf(fid, '%d, ', v(j));
        end
        if (length(val{i}) > 0)
            fprintf(fid, '%d', v(j));
        end
        fprintf(fid, ']\n');
    end
    fclose(fid);
    disp(sprintf('wrote %d masks', length(masks)));
end
