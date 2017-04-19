function visualize_tags(tags, varargin)
%
% visualize_tags(tags [, OPTIONS])
%
% 3d visualization of tags in new format
%
% tags       array with tag poses/errors
% OPTIONS:   'simplemarkers',
%            'scaletags'
%            'frontcol'
%            'backcol'
%
    simpleMarkers = false;
    sc = 1.0;
    overlayName = [];
    frontcol = 'yellow';
    backcol = 'blue';

    ih = ishold;
    for i=1:length(varargin)
        if strcmpi(varargin{i}, 'simplemarkers')
            simpleMarkers = true;
        end
        if strcmpi(varargin{i}, 'scaletags')
            sc = varargin{i+1};
        end
        if strcmpi(varargin{i}, 'frontcol')
            frontcol = varargin{i+1};
        end
        if strcmpi(varargin{i}, 'backcol')
            backcol = varargin{i+1};
        end
    end
    tagids    = tags(:, 1);
    % find 3d points for the tags
    x         = get_points_3d(tags(:,1), tags, sc);
    for idx = 1:size(tagids, 1)
        i = (idx - 1) * 4 + 1;
        % corners of tag
        p = [x(i:i+3, 1), x(i:i+3, 2), x(i:i+3, 3)];
        if simpleMarkers
            drawSimpleTag(p, frontcol);
        else
            drawFancyTag(p, frontcol, backcol);
            labelTag(p, tagids(idx));
        end
        hold on;
    end
    if ih
        hold on;
    else
        hold off;
    end
    xlabel 'X';
    ylabel 'Y';
    axis equal;
end

function drawSimpleTag(p, col)
    fill3(p(:,1), p(:,2), p(:,3), col);
end

function drawFancyTag(p, frontcol, backcol)
%
% draw front face of tag
%
    fill3(p(:,1), p(:,2), p(:,3), frontcol);
    %
    % now draw second quad with different color for the back,
    % slightly displaced
    %
    p1 = (p(2,:)-p(1,:))';
    p2 = (p(3,:)-p(1,:))';
    del = cross(p1, p2) / norm(p1) * 0.05; % displacement between quads
    fill3(p(:,1)+del(1), p(:,2)+del(2), p(:,3)+del(3), backcol);
        
    %
    % black marker top left corner to identify
    %
    del2 = -del*0.5;
    dp = p' - repmat(p(1,:)',1,4);
    pb = (repmat(p(1,:)',1,4) + 0.2 * dp)';
    fill3(pb(:,1)+del2(2), pb(:,2)+del2(2), pb(:,3)+del2(3), 'black');
end

function labelTag(p, id)
%
% put a label in the center
%
        pc = 0.5*(p(2,:) + p(4,:));
        text(pc(1),pc(2),pc(3), ['',int2str(id)]);

end