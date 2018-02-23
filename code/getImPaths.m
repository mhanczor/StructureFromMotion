function [ imPaths ] = getImPaths( folderPath, fileExt )
%GETIMSPATH Returns full path of image files in folderPath
%   Returns all files in the folderPath with the given extension (default
%   jpg).  Doesn't 

%% Check Inputs
if nargin<2
    %default extension
    fileExt = 'jpg';
else
    % prepend ext with '.' if it doesn't exist
    if fileExt(1)~='.'
        fileExt = ['.' fileExt];
    end
end

%% Form list
files = dir(fullfile(folderPath,['*' fileExt]));
imPaths = {files.name}';

%% Concatenate names with folderpath
if ~strcmp(folderPath, '.')
    imPaths = fullfile(folderPath, imPaths);
end
end

