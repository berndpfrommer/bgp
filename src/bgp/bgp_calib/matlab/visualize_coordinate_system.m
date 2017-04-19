function visualize_coordinate_system(TW, NAME, LEN)
%
% visualize_coordinate_system(TW, NAME, LEN)
%
% Visualizes a coordinate system with a 3-legged graph
%
%%--- input ------
% TW       4x4 or 4x3 twist matrix a la [R, T] in homogeneous 4-space
% NAME     text to label the coordinate system with
% LEN      length of axis drawn
%
%
    T = TW(1:3,4);
    R = TW(1:3,1:3);
    
    thickness = 0.02; % relative thickness of axis
    
    %
    % make cylinders for the axis
    % round, but made of nfacets quads.
    %

    nfacets = 10;
    [cx, cy, cz] = cylinder([thickness thickness],nfacets);
    nrings = size(cx, 1);
    rx = [0 0 1;  0 1 0; -1  0 0];
    ry = [1 0 0;  0 0 1;  0 -1 0];
    
    cm = LEN * reshape([cx', cy', cz'], nrings*size(cx,2), 3);
    ncp = size(cm, 1);
    tc  = repmat(T', ncp, 1);

    ax = reshape((R * rx * cm')' + tc,size(cx,2), 3 * nrings);
    ay = reshape((R * ry * cm')' + tc,size(cx,2), 3 * nrings);
    az = reshape((R * cm')' + tc,size(cx,2), 3 * nrings);   
    
    h = ishold;
    surf(ax(:,1:nrings), ax(:,nrings+1:2*nrings), ax(:,2*nrings+1:end), 'FaceColor', 'red', 'LineStyle', 'none');
    hold on;
    surf(ay(:,1:nrings), ay(:,nrings+1:2*nrings), ay(:,2*nrings+1:end), 'FaceColor', 'green', 'LineStyle', 'none');
    surf(az(:,1:nrings), az(:,nrings+1:2*nrings), az(:,2*nrings+1:end), 'FaceColor', 'blue', 'LineStyle', 'none');
    
    %
    % labels 'x', 'y', 'z' on axis
    %
    off = LEN * [0, 0, 1.2]';
    o = [rx * off, ry * off, off];
    txt = R' * o + repmat(T', 3, 1);
    text(txt(1, 1), txt(1,2), txt(1,3), 'x');
    text(txt(2, 1), txt(2,2), txt(2,3), 'y');
    text(txt(3, 1), txt(3,2), txt(3,3), 'z');


    %
    % write the name of the object 
    %
    loff = R' * LEN * [0.1, 0.1, 0.1]' + T;

    text(loff(1), loff(2), loff(3), NAME);
    
    %
    % To create the little cones at the tip of the axis, we
    % make a cylinder with varying diameter. The rings are not really
    % round, but made of nfacets quads.
    %
    relconesz = 0.1;
    [kpx, kpy, kpz] = cylinder([1.5* thickness/relconesz, 0], nfacets);
    nkr = size(kpx,1); % number of cone rings
    km = LEN * relconesz * reshape([kpx', kpy', kpz'], nkr*size(kpx,2), 3);
    km = km + LEN * repmat([0 0 1], size(km, 1), 1);

    %
    % make cone tips for each axis
    %
    nkp = size(km, 1);
    tk  = repmat(T', nkp, 1);
    kx = reshape((R * rx * km')' + tk, size(kpx,2), 3 * nkr);
    ky = reshape((R * ry * km')' + tk, size(kpx,2), 3 * nkr);
    kz = reshape((R * km')' + tk,      size(kpx,2), 3 * nkr);
    surf(kx(:,1:nkr), kx(:,nkr+1:2*nkr), kx(:,2*nkr+1:end), 'FaceColor', 'red', 'LineStyle', 'none');
    surf(ky(:,1:nkr), ky(:,nkr+1:2*nkr), ky(:,2*nkr+1:end), 'FaceColor', 'green', 'LineStyle', 'none');
    surf(kz(:,1:nkr), kz(:,nkr+1:2*nkr), kz(:,2*nkr+1:end), 'FaceColor', 'blue', 'LineStyle', 'none');
    if ~h
        hold off
    end
end
