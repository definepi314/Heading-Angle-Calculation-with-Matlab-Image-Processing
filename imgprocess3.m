    clc;
    close all;
    clear all;
    cam=webcam('USB_Camera');
    set(cam,'Resolution','1920x1080');
    load('cameraparameters.mat');
    im=snapshot(cam);
    im=imcrop(im,[400 470 900 300]);%  - top right -
    im = undistortImage(im, cameraParams);
    im_gr=rgb2gray(im);
    l_th = graythresh(im_gr);
    crescent_moon_image = im2bw(im_gr, 0.265);% crescent moon im_bw
    full_moon_image = im2bw(im_gr, 0.37);% whole circle im_bw1
    blackout_image = im2bw(im_gr, 0.9);%black screen
    intermediate_image = ~(~full_moon_image + ~crescent_moon_image);
  %  imshow(im_bw2 + ~im_bw);
    
    blackout_1 = ~bwareaopen(~blackout_image, 80);
    blackout_2 = bwareaopen(blackout_1, 200);
    s_b=regionprops(~blackout_2,'Centroid');
    pluto_image = intermediate_image + ~crescent_moon_image
    final_pluto_image = ~bwareaopen(~pluto_image, 80);
    
    final_full_moon_image = ~bwareaopen(~full_moon_image, 80);
    pluto = bwareaopen(final_pluto_image, 200);
    full_moon = bwareaopen(final_full_moon_image, 200);
    s=regionprops(~pluto,'Centroid');
    s_1=regionprops(~full_moon,'Centroid');
    
    a=regionprops(~pluto,'Area');
    data_pos = [cat(1,s_1.Centroid) cat(1,a.Area)];
    centers=[]
    ;
if size(data_pos,1)~=0
    data_pos=data_pos(data_pos(:,3)<800,:);
    centers=[data_pos(:,1) data_pos(:,2)];
end

