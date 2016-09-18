clc;
close all;
clear all;
% im=imread('geniusimg.png');
N=10;
cam=webcam('USB_Camera');
set(cam,'Resolution','1280x720')
set(cam,'Contrast',40)
% set(cam,'BacklightCompensation',0)
set(cam,'Brightness',0)
origin_pixel=[368 247];
scale_x=0.63;
scale_y=0.63;
load('cameraparameters.mat')
tic
T=0;
p_data=cell(1,N);
dt=0;
fl_start=0
zigbeeFlag = input('Enter "0" to disable zigbee else "1": ');
pos_cor_x=0;
pos_cor_y=0;
%Error string to send in case of error
errorString = char('z'.*ones(1,12));
if (zigbeeFlag)
    sp = serial('COM10','BaudRate',57600,'DataBits',8);
    % fp.Timeout=3
    fopen(sp);
end

% sp = serial('COM10','BaudRate',115200,'DataBits',8);
% sp.Timeout=3

% fopen(sp);

P_init=zeros(10,2);
h_init=zeros(10,1);
A=80;B=80; a=3 ;b=2;
N=a+b;
for i=1:9
    xa_init=A*cos(2*pi*(i-1)/(a+b));
    ya_init=B*sin(2*pi*(i-1)/(a+b));
    P_init(i,:)=[xa_init ya_init];
    h_init(i)=atan2( B*b*cos(2*pi*(i-1)/(a+b)),A*a*sin(2*pi*(i-1)/(a+b)))
end



% for i=1:3
%     sa=0;%+(i-1)*pi/(a*b);%+2*pi-(pi/b);
%     P_init(i,:)=[A*cos(a*sa) B*sin(b*sa)];
%     h_init(i)=atan2( B*b*cos(b*sa),-A*a*sin(a*sa))
% end
% for i=2
%     sa=2*2*pi/(a+b);
%     P_init(i,:)=[A*cos(a*sa) B*sin(b*sa)];
%     h_init(i)=atan2( B*b*cos(b*sa),-A*a*sin(a*sa))
% end
% for i=N-1:N
%     sa=0+(i-1)*pi/(a*b)-pi/(2*a*b)+2*pi-(pi/b);
%     P_init(i,:)=[A*cos(a*sa) B*sin(b*sa)];
%     h_init(i)=atan2( B*b*cos(b*sa),-A*a*sin(a*sa))
% end
% tic
T_o=cputime;
while(1)
    im=snapshot(cam);
    % imtool(im)
    %     im=imcrop(im,[263.5 76.5 650 439]);orig
    % im=imcrop(im,[255.75 73.25 666 465]);
    im=imcrop(im,[249.75 70.25 673.5 478.5]);
    im = undistortImage(im, cameraParams);
    % imshow(im)
    im_gr=rgb2gray(im);
    %         im_gr=imadjust(im_gr);
    
    l_th = graythresh(im_gr);
    
    im_bw=im2bw(im_gr, 0.2);
%     imshow(im_gr)
    
    im_bw_1 = ~bwareaopen(~im_bw, 200);
    im_bw_2 = bwareaopen(im_bw_1, 400);
    %         subplot(121)
%             imshow(~im_bw_2)
    %     imshow(im)
    s=regionprops(~im_bw_2,'centroid');
    a=regionprops(~im_bw_2,'Area');
    
    %
    data_pos=[cat(1,s.Centroid) cat(1,a.Area)];
    centers=[];
    if size(data_pos,1)~=0
        data_pos=data_pos(data_pos(:,3)<800,:);
        centers=[data_pos(:,1) data_pos(:,2)];
    end
%     hold on
%     if size(centers,1)~=0
%         plot(centers(:,1),centers(:,2),'*')
%     end
% 
%     hold off
    Xbee_data=[];
    for i=1:size(centers,1)
        %     im_ch=imcrop(~im_bw_2,[centers(i,1)-25 centers(i,2)-25 50 50]);
        %     [centers_ch,radii_1] = imfindcircles(im_ch,[10 18]);
        
        if (T>3)
            im_c=imcrop(im,[centers(i,1)-17 centers(i,2)-17 34 34]);
            gr_c=rgb2gray(im_c);
            gr_c2=imadjust(gr_c);
            bw_c=im2bw(gr_c2,graythresh(gr_c2));
           gray= graythresh(gr_c2);
            bk_c=bwareaopen(bw_c,90);
            bk_g=bwareaopen(~bk_c,120);
            bw_c=bw_c+~bk_g;
            bw_c=bw_c.*bk_g;
            
%                         subplot(221)
%                         imshow(gr_c)
%                         subplot(222)
%                         imshow(bw_c)
%                         subplot(223)
%                         imshow(bk_c)
%                         subplot(224)
%                         imshow(bk_g)
            CC=bwconncomp(bw_c);
            SS=regionprops(CC);
            data_c=[];
            
            data_c=[round(cat(1,SS.Centroid)) cat(1,SS.Area)];
            
            if (size(data_c,1)>=2)
                data_c=sortrows(data_c,-3);
                if(data_c(1,3)<60  && data_c(2,3)<40)
                    D=dist(data_c(1,1:2),data_c(2,1:2)');
                    
                    if(D<=13)
                        
                        ID=CC.NumObjects-1;
                        Heading= atan2(-data_c(1,2)+data_c(2,2),data_c(1,1)-data_c(2,1))*180/pi;
                        %                         plot(data_c(2,1),data_c(2,2),'*',data_c(1,1),data_c(1,2),'g*')
                        % ID= hd*3+ld+1;
                        pos_cor_y=(data_c(1,2)+data_c(2,2))/2-17;
                        pos_cor_x=(data_c(1,1)+data_c(2,1))/2-17;
% [pos_cor_x pos_cor_y]
                        Position=[(centers(i,1)-origin_pixel(1))*scale_x -(centers(i,2)-origin_pixel(2))*scale_y];
                        if (fl_start==1)
                            p_data{ID}=[p_data{ID}; [Position Heading ID dt_cpu T] ];
                        end
                        Xbee_data=[Xbee_data; [Position Heading ID ] ];
                    else
                        ID= 999;
                        Heading= 999;
                        Position=[999 999];
                        
                    end
%                             hold off
%                                     getframe()
%                                                     subplot(122)
%                                                     imshow(im_c);
%                                                     hold on
%                                                     plot(data_c(2,1),data_c(2,2),'*')
%                                                     plot(data_c(1,1),data_c(1,2),'g*')
                    if size(Xbee_data,2)~=0
                        Xbee_data=sortrows(Xbee_data,4);
                    end
                end
            end
            
        end
    end
    %     dt= toc
    %     T=T+dt;
    %     tic
    
    T_cpu=cputime;
    dt_cpu=T_cpu-T_o;
    T_o=T_cpu;
    T=T+dt_cpu;
    Timedisp=[T dt_cpu]
    h=[999 999 999 999 999 999 999 999 999 T];
    x=[999 999 999 999 999 999 999 999 999 999];
    y=[999 999 999 999 999 999 999 999 999 999];
    d_init=[];h_err=[];ag_active_id=[];
    for i=1:size(Xbee_data,1)
        ag_i=Xbee_data(i,4);
        x(ag_i)= Xbee_data(i,1);
        y(ag_i)= Xbee_data(i,2);
        h(ag_i)= Xbee_data(i,3);
        d_init(i)=sqrt((x(ag_i)-P_init(ag_i,1)).*(x(ag_i)-P_init(ag_i,1))+( y(ag_i)-P_init(ag_i,2)).*( y(ag_i)-P_init(ag_i,2)))
        h_err(i)=atan2(tan(h_init(ag_i)-h(ag_i)*pi/180),1)*180/pi;
        ag_active_id(i)=ag_i;
    end
    
%     [ag_active_id d_init h_err  ]
    ag_active_id
    
    if size(Xbee_data,1)~=0
        if max(d_init)<20 && max(abs(h_err))<=20
            %         if d_init(1)<12 && h_err(1)<=20
            if fl_start==0
                fl_start=1;
                x_st=[911 912 913 914 915 916 917 918 919 920];
                y_st =[911 912 913 914 915 916 917 918 919 920];
                h_st=[911 912 913 914 915 916 917 918 919 T];
                [pkt_1,pkt_2]= xbee_send(h_st,x_st,y_st);
                if (zigbeeFlag)
                    pause(0.1)
                    
                    fwrite(sp ,pkt_1);
                    pause(0.15)
                    fwrite(sp ,pkt_2);
                    pause(0.1)
                    %                                         fwrite(sp ,pkt_1);
                    %                                         pause(0.15)
                    %                                         fwrite(sp ,pkt_2);
                end
                fl_start
            end
            
        end
        
        
        
        
        %         [x' y' h']
        [pkt_1,pkt_2]= xbee_send(h,x,y);
        if (zigbeeFlag)
            fwrite(sp ,pkt_1);
            tic;
            while toc < 0.05
            end
            fwrite(sp ,pkt_2);
            tic;
%             while toc < 0.05
%             end
            
        end
    end
    
end





















%
% i=6
% im_c=imcrop(im,[centers(i,1)-12 centers(i,2)-12 24 24]);
% im_temp=imcrop(im,[centers(i,1)-6 centers(i,2)-6 12 12])
% im_temp_gr=rgb2gray(im_temp);
% l_th=graythresh(im_temp_gr)
%
% im_c_gr=rgb2gray(im_c);
% im_c_ad=imadjust(im_c_gr);
% im_c_hist=histeq(im_c_ad);
% figure()
% subplot(231)
% imshow(im_c_gr)
% subplot(232)
% imshow(im_c_ad)
% subplot(233)
% imshow(im_c_hist)
%
% subplot(234)
% im_bw1=im2bw(im_c_gr, l_th)
% imshow(im_bw1)
%
% subplot(235)
% im_bw2=im2bw(im_c_ad,l_th)
% imshow(im_bw2)


% imshow(im_c)
% im_c_gr=rgb2gray(im_c);
% im_temp_gr=rgb2gray(im_temp);
% l_th = graythresh(im_temp_gr);
% im_c_bw= im2bw(im_c_gr, l_th);
% im_c_bw_2 = bwareaopen(im_c_bw, 40);


% mu=l_th*255;
%
% rr=(im_c(:,:,1)-mu>im_c(:,:,2)-mu & im_c(:,:,1)-mu> im_c(:,:,3)-mu & im_c(:,:,1)> 115);
% gg=(im_c(:,:,2)>im_c(:,:,1) & im_c(:,:,2)>im_c(:,:,3) & im_c(:,:,2) > 80);
% bb=(im_c(:,:,3)-im_c(:,:,1)>10 &  im_c(:,:,3)-im_c(:,:,2)>10 & im_c(:,:,2)- im_c(:,:,1)> 4 & im_c(:,:,3)>75);




% rr=(im_c(:,:,1)-im_c(:,:,2)>30 & im_c(:,:,1)-im_c(:,:,3)>30   & im_c(:,:,1) < 150 & im_c(:,:,1) > 115);
% gg=(im_c(:,:,2)>im_c(:,:,1) & im_c(:,:,2)>im_c(:,:,3) & im_c(:,:,2) > 80);
% bb=(im_c(:,:,3)-im_c(:,:,1)>10 &  im_c(:,:,3)-im_c(:,:,2)>10 & im_c(:,:,2)- im_c(:,:,1)> 4 & im_c(:,:,3)>75);
% rr=bwareaopen(rr, 5)
% gg=bwareaopen(gg, 5)
% bb=bwareaopen(bb, 5)

% gg=(im_c(:,:,2)>im_c(:,:,1) & im_c(:,:,2)>im_c(:,:,3) & im_c(:,:,2) < 110 &  im_c(:,:,2) > 85);
% bb=(im_c(:,:,3)>im_c(:,:,1) & im_c(:,:,3)>im_c(:,:,2) & im_c(:,:,3) < 110 &  im_c(:,:,3) > 85);
% figure()
% subplot(221)
% imshow (im_c)
% subplot(222)
% imshow(rr)
% title('rr')
% subplot(223)
% imshow(gg)
% title('gg')
% subplot(224)
% imshow(bb)
% title('bb')
%  imshow(im_c_bw)
