% Load the two images
image1 = imread('IMG_20230420_004418.jpg');
image2 = imread('IMG_20230420_004411.jpg');

% Check that the two images are the same size
if size(image1) ~= size(image2)
    error('The two images must be the same size!');
end

% Compute the absolute difference between the two images
difference = abs(image1 - image2);

% Display the difference image
figure;
imshow(image1)
figure;
imshow(image2)
figure;
imshow(difference);
title('Absolute difference between the two images');
