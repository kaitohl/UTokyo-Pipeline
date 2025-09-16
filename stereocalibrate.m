function stereoParams= stereocalibrate(basePath, folder, method, ...
    resolution, squareSize)

% Construct left and right folder paths
leftFolder = fullfile(basePath, folder, method, resolution, 'left');
rightFolder = fullfile(basePath, folder, method, resolution, 'right');

% Get sorted list of PNG files
leftImages = dir(fullfile(leftFolder, '*.png'));
rightImages = dir(fullfile(rightFolder, '*.png'));

% Check if same number of left and right images
if length(leftImages) ~= length(rightImages)
    warning('Number of left and right images does not match for %s %s', method, resolution);
    return;
end

% Construct file paths
imageFileNames1 = fullfile({leftImages.folder}, {leftImages.name});
imageFileNames2 = fullfile({rightImages.folder}, {rightImages.name});

% Detect checkerboard corners
detector = vision.calibration.stereo.CheckerboardDetector();
minCornerMetric = 0.15;
[imagePoints, imagesUsed] = detectPatternPoints(detector, imageFileNames1, imageFileNames2, ...
    'MinCornerMetric', minCornerMetric);

% If no valid image pairs found, skip
if nnz(imagesUsed) < 2
    warning('Not enough valid images detected for %s %s', method, resolution);
    return;
end

% Generate world points
worldPoints = generateWorldPoints(detector, 'SquareSize', squareSize);

% Get image size
sampleImage = imread(imageFileNames1{find(imagesUsed, 1)});
[mrows, ncols, ~] = size(sampleImage);

% Calibrate stereo camera
[stereoParams, ~, ~] = estimateCameraParameters(...
    imagePoints, worldPoints, ...
    'EstimateSkew', false, ...
    'EstimateTangentialDistortion', true, ...
    'NumRadialDistortionCoefficients', 2, ...
    'WorldUnits', 'millimeters', ...
    'ImageSize', [mrows, ncols]);

% Print result
meanError = (stereoParams.MeanReprojectionError);
fprintf('Reprojection error for %s %s: %.4f pixels\n', method, resolution, meanError);
fprintf('Baseline value: %.4f\n', stereoParams.PoseCamera2.Translation(1,1))


end