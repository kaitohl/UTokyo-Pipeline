% Create checkerboard

pixperinch = 264;

mmperinch = 25.4;

pixpermm = pixperinch/mmperinch;

desiredpixpersquare = 208;

mmpersquare = desiredpixpersquare/pixpermm;

y = 7;

x = 10;

J_full = checkerboard(desiredpixpersquare, ceil(x/2) , ceil(y/2)) > 0.5;

J = J_full(1:x*desiredpixpersquare,1:y*desiredpixpersquare);

imshow(J)


dpi = 264; % native iPad PPI

figure('Units','inches','Position',[0 0 size(J,2)/dpi size(J,1)/dpi]);
imshow(J, 'InitialMagnification', 'fit');
axis off;
set(gca, 'Position', [0 0 1 1]);

% Export as PDF (best for preserving size)
print('checkerboard_true_size', '-dpdf', ['-r' num2str(dpi)]);