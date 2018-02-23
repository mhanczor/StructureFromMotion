function [ newRGB ] = removeBG( rgbImage, vThresh, sThresh )

if nargin<2
    vThresh = 0.5;
end

if nargin < 3
    sThresh = 0.25;
end
rgbImage = imsharpen(rgbImage);
% Convert RGB image into HSV color space.
hsvImage = rgb2hsv(rgbImage);
% Extract individual H, S, and V images.
h = hsvImage(:,:, 1);
s = hsvImage(:,:, 2);
v = hsvImage(:,:, 3);
% Threshold to find vivid colors.
nPixels = numel(v);
mask = (v > vThresh) & (s < sThresh);
mask = imfill(~mask, 'holes');
mask = ~bwareaopen(mask, round(0.1*nPixels));

se = strel('disk',30);
mask = imclose(mask, se);

% Make image white in mask areas:
h(mask) = 0;
s(mask) = 0;
v(mask) = 1;
% Convert back to RGB
hsvImage = cat(3, h, s, v);
newRGB = hsv2rgb(hsvImage);
end

