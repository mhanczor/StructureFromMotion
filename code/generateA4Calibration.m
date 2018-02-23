function generateA4Calibration(destFilename)
    h = figure;
    nBoxes = 10;
    squareSideLength = 2;
    
    imPos = [0, 0, squareSideLength*nBoxes, squareSideLength*nBoxes];
    im = checkerboard(nBoxes);
    im = imresize(im, 50, 'nearest');
    im = im < 0.5;
    imshow(im);
    set(h, ...
        'PaperUnits','centimeters', ...
        'PaperPosition', imPos);
    print(h, destFilename, '-dpdf', '-r0')
end