function tags = make_tags()
% tags = make_tags()
%
% 2017 Bernd Pfrommer
%    
% creates array of tags for test case
%
% tags = n x 14 array, where n = number of tags (about 191)
%
% The columns have the following meaning:
% 1     2          3-5        6-8       9-11      12-14
% tagid tagsize[m] position,  rotation  posnoise  rotation noise
%
% The rotation vectors associated with the tags transform into
% the world frame:
%
%  X_world = R * X_tag + T;
%
% example usage:
%
% tags = make_tags();
% visualize_tags(tags);
%
    RA = vrrotvec2mat([1 0 0 pi]);
    tags = make_right_wall(RA);
    tags = [tags; make_back_wall(RA)];
    tags = [tags; make_left_wall(RA)];
    tags = [tags; make_front_wall(RA)];
end

function tags = make_front_wall(RA)
    ncols  = 6;
    nrows  = 8;
    n      = nrows * ncols;

    ids   = [48:1:95]';
    skip_ids = [74, 75, 66, 67];
    
    
    if length(ids) ~= n
        error ('id length is off!')
    end
    zstart    = 0.03175;
    xstart    = 0.0;
    ystart    = 0.43815;
    tagsize   = 0.03175;
    tagcolsep = 0.076835;
    tagrowsep = 0.064135;
    % rotation of -90 degrees around x
    R   = (RA*vrrotvec2mat([1,0,0,pi/2]) * vrrotvec2mat([0,0,1,-pi/2]))';
    Rtag = vrrotvec2mat([0,0,1,pi]);
    tags = make_wall(xstart, ystart, zstart, tagsize, tagcolsep, ...
                     tagrowsep, ncols, nrows, R, Rtag, ids, skip_ids);
end

function tags = make_left_wall(RA)
%
% the wall faced by cam2
%
    ncols  = 6;
    nrows  = 8;
    n      = nrows * ncols;

    ids   = [183:-1:136]';
    skip_ids = [156, 157, 164, 165, 174, 175, 182, 183];
    
    
    if length(ids) ~= n
        error ('id length is off!')
    end
    zstart    = 0.03175;
    xstart    = 0.4333875;
    ystart    = 0.48895;
    tagsize   = 0.0301625;
    tagcolsep = 0.076835;
    tagrowsep = 0.06440714;
    % rotation of -90 degrees around x
    R   = (RA*vrrotvec2mat([1,0,0,pi/2]))';
    tags = make_wall(xstart, ystart, zstart, tagsize, tagcolsep, ...
                     tagrowsep, ncols, nrows, R, eye(3), ids, skip_ids);
end

function tags = make_back_wall(RA)
    ncols  = 6;
    nrows  = 8;
    n      = nrows * ncols;

    ids   = [47:-1:0]';
    skip_ids = [20, 21, 28, 29];
    
    
    if length(ids) ~= n
        error ('id length is off!')
    end
    zstart    = 0.0445;
    xstart    = 0.48895;
    ystart    = 0.0567;
    tagsize   = 0.03175;
    tagcolsep = 0.0747;
    tagrowsep = 0.0619125;
    % rotation of 90 degrees around z, followed by 90 around x
    R   = (RA*vrrotvec2mat([1,0,0,pi/2]) * vrrotvec2mat([0,0,1,pi/2]))';
    tags = make_wall(xstart, ystart, zstart, tagsize, tagcolsep, ...
                     tagrowsep, ncols, nrows, R, eye(3), ids, skip_ids);
end

function tags = make_right_wall(RA)
    ncols  = 6;
    nrows  = 8;
    n      = nrows * ncols;

    ids   = [231:-1:184]';
    skip_ids = [204, 205, 212, 213, 218];
    
    
    if length(ids) ~= n
        error ('id length is off!')
    end
    zstart    = 0.03175;
    xstart    = 0.047625;
    ystart    = 0.00635;
    tagsize   = 0.03175;
    tagcolsep = 0.0765175;
    tagrowsep = 0.06440714;
    % rotation of 90 degrees around x, followed by 180degrees around z
    R    = (RA*vrrotvec2mat([1,0,0,pi/2]) * vrrotvec2mat([0,0,1,-pi]))';
    tags = make_wall(xstart, ystart, zstart, tagsize, tagcolsep, ...
                     tagrowsep, ncols, nrows, R, eye(3), ids, skip_ids);
end


function tags = make_wall(xstart, ystart, zstart, tagsize, tagcolsep, ...
                          tagrowsep, ncols, nrows, Rwall, Rtag, ids, skip_ids)
    col_dir = Rwall * [-1 0 0]';
    row_dir = Rwall * [0  1 0]';
    
    X0    = [xstart, ystart, zstart]';
    wRo   = Rwall * Rtag;
    rvec  = vrrotmat2vec(Rwall*Rtag);
    rvec  = rvec(1:3)*rvec(4);
    pos   = [];
    rot   = [];
    n     = length(ids);
    for i = 0:(n-1)
        row = mod(i, nrows);
        col = floor(i/nrows);
        dv  = X0 + col_dir * tagcolsep * col + row_dir * tagrowsep * row;
        pos = [pos; dv'];
        rot = [rot; rvec];
    end
    % pose noise is in object (tag) coordinates!
    posnoise = repmat(abs((Rtag * [2e-3; 2e-3; 1e-3])'), n, 1);
    rotnoise = 1e-4 * ones(n, 3);
    sizes = tagsize * ones(n,1);
    tagstmp  = [ids, sizes, pos, rot, posnoise, rotnoise];

    tags = [];
    for i=1:n
        if ~ismember(tagstmp(i,1), skip_ids)
            tags = [tags; tagstmp(i,:)];
        end
    end
end
