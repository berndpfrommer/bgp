# Matlab helper tools


## generate test data

To generate test tag poses:

    tags = make_tags();
	visualize_tags(tags);
	write_tags(tags, '../config/tag_poses.yaml');

## analyze test data

To analyze data produced by the calibration tool (assume that
calibration tool outputs into the ../test directory)

	%
	% plot reprojection error
	%
    plot_reprojection_data(textread('../test/reproj.txt'));
	
	% 
	% plot optimized tags vs original tags
	%
	visualize_tags(make_tags()); hold on; visualize_tags(textread('../test/tag_poses.txt'), 'simplemarkers', 'frontcol', 'red'); hold off;

	%
	% visualize camera poses vs original tags
	%
	visualize_tags(make_tags()); hold on; visualize_cameras(textread('../test/cam_poses.txt')); hold off;
    
	
