clc;
clear;
close all;

% Define base path
basePath = '/Users/kaitohara-lee/Library/CloudStorage/OneDrive-Queen''sUniversity/School/UTokyo/Project';
folder = 'Calibration_20250704';
method = 'BL';
resolution = '1280x720';
squareSize = 10.006;  % in millimeters
handeyefolder = 'handeye2';
testfolder = 'handeyetest3';

%% Camera Calibrate (Not Rectified)
stereoParams = stereocalibrate(basePath,handeyefolder,method,resolution,squareSize);

% Export for OpenCV
[intrinsicMatrix1,distortionCoefficients1,intrinsicMatrix2, ...
   distortionCoefficients2,rotationOfCamera2,translationOfCamera2] =... 
   stereoParametersToOpenCV(stereoParams);

cameraMatrix1 = intrinsicMatrix1;
distCoeffs1 = distortionCoefficients1;
cameraMatrix2 = intrinsicMatrix2;
distCoeffs2 = distortionCoefficients2;
R = rotationOfCamera2;
T = translationOfCamera2;

save('stereoParams_handeye2.mat', ...
    'cameraMatrix1', ...
    'distCoeffs1', ...
    'cameraMatrix2', ...
    'distCoeffs2', ...
    'R', ...
    'T');


% %% Hand-Eye calibrate (Rectified)
% clc;
% clear;
% close all;
% 
% [cameraToEndEffectorTform,intrinsicsLeftRect] = handeye(stereoParams, basePath,handeyefolder,method, ...
%     resolution,squareSize);
% 
% %% Validation
% % Build mocap folder path
% lefttestfolder = fullfile(basePath, testfolder, method, resolution, 'left');
% righttestfolder = fullfile(basePath, testfolder, method, resolution, 'right');
% 
% % Get sorted list of PNG files
% lefttestImages = imageDatastore(lefttestfolder, 'FileExtensions', '.png');
% righttestImages = imageDatastore(righttestfolder, 'FileExtensions', '.png');
% 
% % Read in raw images
% I1test = readimage(lefttestImages,1);
% I2test = readimage(righttestImages,1);
% 
% % Rectify images
% [J1test,J2test,Q,camera1,camera2] = rectifyStereoImages(I1test,I2test,stereoParams);
% 
% % Detect checkerboard and generate new checkerboard
% [imagePoints, boardSize] = detectCheckerboardPoints(J1test);
% worldPoints3D = generateCheckerboardPoints(boardSize, squareSize);
% 
% % Compute extrinsics with rectified intrinsics
% camExtrinsics = estimateExtrinsics(imagePoints, worldPoints3D, intrinsicsLeftRect);
% 
% %% Plot
% % Show rectified left image
% % Left camera pose
% camPoseL = extr2pose(camExtrinsics);
% 
% % Compute baseline
% baseline = -1 / Q(4,3);
% T_baseline = eye(4);
% T_baseline(1,4) = baseline;
% Tbase = rigidtform3d(T_baseline);
% Tbaseinv = invert(Tbase);
% camPoseR = rigidtform3d(camPoseL.A * Tbaseinv.A);
% 
% % Checkerboard corners
% cornerCoords = [worldPoints3D, zeros(size(worldPoints3D,1),1)];
% 
% figure;
% plotCamera('AbsolutePose', camPoseL, 'Size', 20, 'Color', [0 0 1], 'Label', 'Left Cam');
% hold on;
% plotCamera('AbsolutePose', camPoseR, 'Size', 20, 'Color', [1 0 0], 'Label', 'Right Cam');
% pcshow(cornerCoords, 'VerticalAxisDir','down','MarkerSize',40);
% 
% xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
% grid on; axis equal;
% title('Stereo Camera Visualization');
% legend('Checkerboard');
% 
% % Overlay text labels at each checkerboard corner
% for i = 1:size(cornerCoords,1)
%     text(cornerCoords(i,1), cornerCoords(i,2), ...
%         sprintf('(%.1f, %.1f, %.1f)', ...
%         cornerCoords(i,1), cornerCoords(i,2)), ...
%         'FontSize',5, 'Color','w');
% end
% 
% % === Plot cam1 coordinate axes ===
% origin = camPoseL.Translation;
% R = camPoseL.Rotation;   % 3x3 rotation matrix
% 
% axisLength = 30; % length of axes arrows in mm
% 
% quiver3(origin(1), origin(2), origin(3), ...
%     R(1,1)*axisLength, R(2,1)*axisLength, R(3,1)*axisLength, ...
%     'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName','X-axis');
% quiver3(origin(1), origin(2), origin(3), ...
%     R(1,2)*axisLength, R(2,2)*axisLength, R(3,2)*axisLength, ...
%     'g', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName','Y-axis');
% quiver3(origin(1), origin(2), origin(3), ...
%     R(1,3)*axisLength, R(2,3)*axisLength, R(3,3)*axisLength, ...
%     'b', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName','Z-axis');
% 
% legend('show');
% 
% %% Mocap Data
% 
% % Build mocap file path
% mocapdir = fullfile(basePath, testfolder, "mocapimages");
% 
% % list csv files
% csvFiles = dir(fullfile(mocapdir, '*.csv'));
% numPosestest = numel(csvFiles);
% 
% % Check if any csv exists
% if isempty(csvFiles)
%     error('No CSV files found in %s', mocapdir);
% end
% 
% % Define transformation matrices here
% T_tiptoworld(numPosestest,1) = rigidtform3d;
% T_markertoworld(numPosestest,1) = rigidtform3d;
% T_tiptocam(numPosestest,1) = rigidtform3d;
% T_tiptoboard(numPosestest,1) = rigidtform3d;
% T_camtomarker = cameraToEndEffectorTform;
% 
% for f = 1:numPosestest
%     % Get name of csv and read
%     fname = fullfile(csvFiles(f).folder, csvFiles(f).name);
%     tbl = readtable(fname, 'VariableNamingRule','preserve');
% 
%     % Extract marker (scope) rotation (XYZ) and translation (XYZ)
%     % Average out all the frames
%     rxm_raw = tbl.("Rigid Body");
%     rxm = rxm_raw(~isnan(rxm_raw));
%     avg_rxm = mean(rxm);
% 
%     rym_raw = tbl.("Rigid Body_1");
%     rym = rym_raw(~isnan(rym_raw));
%     avg_rym = mean(rym);
% 
%     rzm_raw = tbl.("Rigid Body_2");
%     rzm = rzm_raw(~isnan(rzm_raw));
%     avg_rzm = mean(rzm);
% 
%     txm_raw = tbl.("Rigid Body_3");
%     txm = txm_raw(~isnan(txm_raw));
%     avg_txm = mean(txm);
% 
%     tym_raw = tbl.("Rigid Body_4");
%     tym = tym_raw(~isnan(tym_raw));
%     avg_tym = mean(tym);
% 
%     tzm_raw = tbl.("Rigid Body_5");
%     tzm = tzm_raw(~isnan(tzm_raw));
%     avg_tzm = mean(tzm);
% 
%     % Convert to homogeneous transform
%     R_marker = eul2rotm(deg2rad([avg_rxm, avg_rym, avg_rzm]), 'XYZ');
%     t_marker = [avg_txm; avg_tym; avg_tzm];
%     T_markertoworld(f) = rigidtform3d(R_marker, t_marker);
% 
%     % Extract forcep rotation (XYZ) and translation (XYZ)
%     % Average out all the frames
%     rx_raw = tbl.("Rigid Body_6");
%     rx = rx_raw(~isnan(rx_raw));
%     avg_rx = mean(rx);
% 
%     ry_raw = tbl.("Rigid Body_7");
%     ry = ry_raw(~isnan(ry_raw));
%     avg_ry = mean(ry);
% 
%     rz_raw = tbl.("Rigid Body_8");
%     rz = rz_raw(~isnan(rz_raw));
%     avg_rz = mean(rz);
% 
%     tx_raw = tbl.("Rigid Body_9");
%     tx = tx_raw(~isnan(tx_raw));
%     avg_tx = mean(tx);
% 
%     ty_raw = tbl.("Rigid Body_10");
%     ty = ty_raw(~isnan(ty_raw));
%     avg_ty = mean(ty);
% 
%     tz_raw = tbl.("Rigid Body_11");
%     tz = tz_raw(~isnan(tz_raw));
%     avg_tz = mean(tz);
% 
%     % Convert to homogeneous transform
%     R = eul2rotm(deg2rad([avg_rx, avg_ry, avg_rz]), 'XYZ');
%     t = [avg_tx; avg_ty; avg_tz];
%     T_tiptoworld(f) = rigidtform3d(R, t);
% end
% 
% %% Calculate tip to camera tf matrix
% 
% % Need to move the tip points to the checkerboard space. 
% % camExtrinsics is a tf from world (board) to camera space
% 
% T_camtochecker = invert(camExtrinsics);
% T_markertocam = invert(T_camtomarker);
% tipInCheckerAll = zeros(numPosestest,3);
% 
% for f = 1:numPosestest
% 
%     % Invert markertoworld and camtomarker
%     T_worldtomarker = invert(T_markertoworld(f));
%     % Build full transform
%     T_tiptoboard(f) = rigidtform3d( ...
%          T_camtochecker.A * T_markertocam.A * T_worldtomarker.A);
%     % Apply transform on original tip points
%     tipInCheckerAll(f,:) = transformPointsForward(T_tiptoboard(f), ...
%         T_tiptoworld(f).Translation);
% 
% end
% 
% % Indices of the touched corners
% touchedIdx = [1, 49, 6, 54, 28];
% 
% % Extract touched corners
% touched_corners = cornerCoords(touchedIdx, :);
% 
% % Compute errors
% errors = sqrt(sum((tipInCheckerAll - touched_corners).^2,2));
% 
% % Calculate the mean error across all poses
% meanError = mean(errors);
% fprintf('Mean error across all touched points: %.2f mm\n', meanError);
% 
% 
% %% Visualize 3D
% 
% figure; hold on;
% 
% % Plot the left camera 
% plotCamera('AbsolutePose', camPoseL, 'Size', 20, 'Color', [0 0 1], 'Label', 'Left Cam');
% % Plot the right camera
% plotCamera('AbsolutePose', camPoseR, 'Size', 20, 'Color', [1 0 0], 'Label', 'Right Cam');
% % Plot the checkerboard
% pcshow(cornerCoords, 'VerticalAxisDir','down','MarkerSize',40);
% 
% % Highlight touched corners (cyan)
% scatter3(touched_corners(:,1), touched_corners(:,2), touched_corners(:,3), ...
%          100, 'c', 'o', 'LineWidth',1.5);
% 
% % Plot calculated tip positions (red X)
% scatter3(tipInCheckerAll(:,1), tipInCheckerAll(:,2), tipInCheckerAll(:,3), ...
%          100, 'r', 'x', 'LineWidth',2);
% 
% % Connect each tip to its touched corner with black line + label error
% for i = 1:numPosestest
%     plot3([touched_corners(i,1), tipInCheckerAll(i,1)], ...
%           [touched_corners(i,2), tipInCheckerAll(i,2)], ...
%           [touched_corners(i,3), tipInCheckerAll(i,3)], 'w-');
% 
%     mid = (touched_corners(i,:) + tipInCheckerAll(i,:))/2;
%     text(mid(1), mid(2), mid(3), sprintf('%.1f mm', errors(i)), ...
%          'Color','m','FontSize',10,'FontWeight','bold');
% end
% 
% xlabel('X (mm)');
% ylabel('Y (mm)');
% zlabel('Z (mm)');
% title('3D Validation of Forceps Tip vs Checkerboard Corners');
% grid on; axis equal;
% legend('All checkerboard corners','Touched corners','Calculated tips');
