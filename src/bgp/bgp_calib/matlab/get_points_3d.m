function POINTS = get_points_3d(SOMETAGIDS, ALLTAGS, varargin)
%
%   POINTS = get_points_3d(SOMETAGIDS, ALLTAGS [, SCALE])
%
% find 3d points for tags provided in SOMETAGIDS, using
% the tag location information provided in ALLTAGS. Scales up
% tag size by optional factor SCALE
%
    nids   = length(SOMETAGIDS);
    nodes = [[-1, 1, 0]', [1, 1, 0]', [1, -1, 0]', [-1, -1, 0]'];
    POINTS = zeros(nids * 4, 3);
    sc = 1.0;
    if length(varargin) > 0
        sc = varargin{1};
    end
    for i = 1:nids
        % search for tag among known tags
        idx = find(ALLTAGS(:, 1) == SOMETAGIDS(i));
        assert (~isempty(idx), 'unknown tag id: %d', SOMETAGIDS(i));
        assert (size(idx, 1) <= 1, ['multiple tags with id: %d, check ' ...
                            'tags array!'], SOMETAGIDS(i));
        id     = ALLTAGS(idx, 1);
        tsize  = ALLTAGS(idx, 2); 
        pos    = ALLTAGS(idx, 3:5);
        rotvec = ALLTAGS(idx, 6:8);
        rvec   = [1,0,0,0];
        if (norm(rotvec) > 1e-8)
            rvec = [rotvec(1:3)/norm(rotvec), norm(rotvec)];
        end
        m      = vrrotvec2mat(rvec);
        nrot   = (m * nodes)' * tsize * sc * 0.5; % world units are 0.5 * edge
                                                  % length
        for k = 1:4
            nrot(k, :) = nrot(k, :) + pos;
        end
        POINTS((1 + (i-1)*4):(i*4), :) = nrot;
    end
