sourceDir = 'C:\Users\Brian\Desktop\cv\voltmeter_front';
destDir = 'C:\Users\Brian\Desktop\cv\voltmeter_front_no_bg';

imSet = imageSet(sourceDir);

images = cell(1, imSet.Count);
parfor i = 1:imSet.Count
    im = read(imSet, i);
    
    [pathstr,name,ext] = fileparts(imSet.ImageLocation{i});
    noBackground = removeBG(im);
    
    destName = fullfile(destDir, [name,ext]);
    
    imwrite(noBackground, destName);
end


%% Touch up 7,12,13,
touchupSet = [7,12,13];
for i = touchupSet
    im = read(imSet, i);
    
    [pathstr,name,ext] = fileparts(imSet.ImageLocation{i});
    noBackground = removeBG(im, 0.5, 0.2);
    
    destName = fullfile(destDir, [name,ext]);
    
    imwrite(noBackground, destName);
end
