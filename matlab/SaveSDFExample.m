% Example to save sdf as gpmp2 datatype
% @author Mustafa Mukadam
% @date   May 4, 2017

close all
clear

%% dataset
dataset = generate3Ddataset('KUKADeskDataset');
origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
cell_size = dataset.cell_size;

% init sdf
disp('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size);
disp('calculating signed distance field done');

dlmwrite('./dataset/kukaDeskDataset.txt', origin, 'delimiter', ' ');
dlmwrite('./dataset/kukaDeskDataset.txt', cell_size, 'delimiter', ' ','-append');
dlmwrite('./dataset/kukaDeskDataset.txt', size(field), 'delimiter', ' ', '-append');
for i = 0:99
    dlmwrite('./dataset/kukaDeskDataset.txt', field(:,:,100-i), 'delimiter', ' ', '-append');
end

%% display SDF
plotSignedDistanceField3D(field, origin, cell_size)