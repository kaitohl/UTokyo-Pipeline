% iPad screen resolution
screen_width_px = 2388;
screen_height_px = 1668;

% Checkerboard specs
square_px = 104;
cols = 10;
rows = 7;
board_width = cols * square_px;
board_height = rows * square_px;

% Ensure checkerboard is large enough
% Generate more than needed, then crop
tiles_x = ceil(board_width / (2*square_px)) + 1;
tiles_y = ceil(board_height / (2*square_px)) + 1;

% Generate and crop to exact checkerboard size
J_board = checkerboard(square_px, tiles_y, tiles_x) > 0.5;
J_board = J_board(1:board_height, 1:board_width);

% Create full white canvas
J_full = ones(screen_height_px, screen_width_px);  % 1 = white

% Center the checkerboard on the canvas
start_x = floor((screen_width_px - board_width) / 2) + 1;
start_y = floor((screen_height_px - board_height) / 2) + 1;

% Insert checkerboard
J_full(start_y:start_y + board_height - 1, ...
       start_x:start_x + board_width - 1) = J_board;

% Save as PNG
imwrite(J_full, 'checkerboard_2388x1668.png');
