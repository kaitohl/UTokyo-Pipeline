function [cameraToEndEffectorTform,intrinsicsLeftRect] = handeye(stereoParams,basePath, ...
    handeyefolder,method,resolution,squareSize)
%% Stereo Rectify

% Construct left and right folder paths
leftFolder = fullfile(basePath, handeyefolder, method, resolution, 'left');
rightFolder = fullfile(basePath, handeyefolder, method, resolution, 'right');

% Get sorted list of PNG files
leftImages = imageDatastore(leftFolder, 'FileExtensions', '.png');
rightImages = imageDatastore(rightFolder, 'FileExtensions', '.png');

% Construct file paths
I1 = readimage(leftImages,1);
I2 = readimage(rightImages,1);

% Rectify
[J1,J2,reprojmat,cam1,cam2,R1,R2] = rectifyStereoImages(I1,I2,stereoParams);

% Create camera 1 (left) intrisic values
K1 = cam1(:,1:3);
imageSize = size(J1,[1 2]);
intrinsicsLeftRect = cameraIntrinsics([K1(1,1) K1(2,2)], [K1(1,3) K1(2,3)], imageSize);
%% Hand-eye calibrate

% Number of poses and create camExtrinsic variable
numPoses = length(leftImages.Files);
camExtrinsics(numPoses,1) = rigidtform3d;

% Initialize log variables
badImages = [];  
warningCount = 0;

% Process each pair
for k = 1:numPoses

    % Read and rectify
    I1 = readimage(leftImages, k);
    I2 = readimage(rightImages, k);
    % Use left rectified image only
    [J1, ~] = rectifyStereoImages(I1, I2, stereoParams);

    % Reset last warning
    lastwarn('');

    % Detect checkerboard
    [imagePoints, boardSize] = detectCheckerboardPoints(J1);

    % Check for warning
    [warnMsg, warnId] = lastwarn;
    if ~isempty(warnMsg)
        warningCount = warningCount + 1;
        badImages(end+1) = k; %#ok<AGROW>
        fprintf('⚠️ Warning for image %d: %s\n', k, warnMsg);
        continue;  % skip extrinsic estimation for this image
    end

    % If detection failed completely or nan
    if isempty(imagePoints) || any(isnan(imagePoints(:)))
        fprintf('❌ Invalid checkerboard in image %d (empty or NaN). Skipping.\n', k);
        badImages(end+1) = k; %#ok<AGROW>
        continue;
    end

    % Generate checkerboard world points (on z=0 plane)
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);

    % Compute extrinsics with rectified intrinsics
    camExtrinsics(k) = estimateExtrinsics(imagePoints, worldPoints, intrinsicsLeftRect);
end

% Identify valid indices
allIndices = 1:numPoses;
goodIndices = setdiff(allIndices, badImages);

% Reduce camExtrinsics to only valid poses
camExtrinsics = camExtrinsics(goodIndices);

fprintf('Final number of valid extrinsics: %d\n', numel(camExtrinsics));

% Summary
fprintf('\nTotal images with warnings or failed detection: %d\n', numel(badImages));
disp('Indices of problematic images:');
disp(badImages);

%% Process Mocap side

% Read in mocap files
mocapdir = fullfile(basePath, handeyefolder, "mocapimages");
csvFiles = dir(fullfile(mocapdir, '*.csv'));

% Check if empty
if isempty(csvFiles)
    error('No CSV files found in %s', mocapdir);
end

% initialize variable to store scope pose
Hg(numPoses,1) = rigidtform3d;

for f = 1:numel(csvFiles)
    
    % Read csv file
    fname = fullfile(csvFiles(f).folder, csvFiles(f).name);
    tbl = readtable(fname, 'VariableNamingRule','preserve');

    % Rotation X
    rx_raw = tbl.("Rigid Body");
    rx = rx_raw(~isnan(rx_raw));
    avg_rx = mean(rx);

    % Rotation Y
    ry_raw = tbl.("Rigid Body_1");
    ry = ry_raw(~isnan(ry_raw));
    avg_ry = mean(ry);

    % Rotation Z
    rz_raw = tbl.("Rigid Body_2");
    rz = rz_raw(~isnan(rz_raw));
    avg_rz = mean(rz);

    % Translation X
    tx_raw = tbl.("Rigid Body_3");
    tx = tx_raw(~isnan(tx_raw));
    avg_tx = mean(tx);

    % Translation Y
    ty_raw = tbl.("Rigid Body_4");
    ty = ty_raw(~isnan(ty_raw));
    avg_ty = mean(ty);

    % Translation Z
    tz_raw = tbl.("Rigid Body_5");
    tz = tz_raw(~isnan(tz_raw));
    avg_tz = mean(tz);

    % Convert average rotation to matrix
    R = eul2rotm(deg2rad([avg_rx, avg_ry, avg_rz]), 'XYZ');
    t = [avg_tx; avg_ty; avg_tz];

    % Store the homogeneous transform
    Hg(f) = rigidtform3d(R, t);

end

Hg = Hg(goodIndices);
fprintf('Collected %d poses from %d CSV files\n', numel(Hg), numel(csvFiles));

%% Compute Extrinsics
config = "moving-camera";
cameraToEndEffectorTform = estimateCameraRobotTransform(camExtrinsics,Hg,config);

%% Validate calibration
