clc; clear all; close all;

delete('tmp.pgm');
filelist = dir('*.pgm');
f2 = fopen('list.txt', 'w');
if f2 == -1
    error('Could not create file list.txt.');
end

fprintf(f2,'%d\n',length(filelist));

for i=1:length(filelist)
    img = imread(filelist(i).name);
    if size(img,3) == 3
        img = rgb2gray(img);
    end
    [rows, cols] = size(img);
    f = fopen('tmp.pgm', 'w');
    if f == -1
        error('Could not create file tmp.pgm.');
    end
    fprintf(f, 'P5\n%d\n%d\n255\n', cols, rows);
    fwrite(f, img', 'uint8');
    fclose(f);
    if isunix
        command = '!./sift ';
    else
        command = '!siftWin32 ';
    end 
    L = length(filelist(i).name);
    command = [command ' <tmp.pgm >' filelist(i).name(1:L-4) '.key']; 
    eval(command);
    fprintf(f2,'%s %s\n',filelist(i).name,...
        [filelist(i).name(1:L-4) '.key']);
end
delete('tmp.pgm');
fclose(f2);






% [image, descrips, locs] = sift(imname);
%     
%     [temp ,s_loc ]   = mysort( descrips, locs );
%     tsize = max(size(temp(:,1)));
%     
%     if(tsize < descriptor_max_num  )
%         bike_test_set(nstart+1:nstart+tsize,:) = temp(1:tsize,:);
%         nstart = nstart  + tsize;
%     else
%         bike_test_set(nstart+1:nstart+descriptor_max_num,:) = temp(1:descriptor_max_num,:);
%         nstart = nstart +descriptor_max_num;
%     end