P=imread('p (5).png');
P=rect_xg(P);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%畸变处理
BW=im2bw(P,0.8);
imshow(BW);

k=150;
BW = bwareaopen(BW,k);
BW = imfill(BW,'holes');
BW = edge(BW,'canny');
%-------------------------
% PPP=imread('B (2).png');     %AAAAAAAAAAAAAAAAAAAAA
% P=rgb2gray(PPP);
% level=graythresh(P);
% 
% level=level+(1-level)/2.5;      % A3 A6 A8    B1    s2            1.5 
%                                 % A7 A10      B2  B3 B4 B6    s1  2        B5 3
% 
% BW=im2bw(P,level);
% k=50;
% BW = bwareaopen(BW,k);
% imshow(BW);
% se1=strel('disk',5);%这里是创建一个半径为5的平坦型圆盘结构元素
% BW=imclose(BW,se1);
% k=20;
% BW = bwareaopen(BW,k);
% BW = edge(BW,'canny');

%--------------------------
% P=imread('d.png');        %单独二值化
% BW=im2bw(P,0.01);
% k=30;
% BW = bwareaopen(BW,k);
%-----------------------------

[H,T,R] = hough(BW,'RhoResolution',1,'ThetaResolution',0.2);

figure,imshow(imadjust(mat2gray(H)),'XData',T,'YData',R,...
      'InitialMagnification','fit');

xlabel('\theta'), ylabel('\rho');
axis on, axis normal,hold on;
colormap(hot);

PP= houghpeaks(H,4,'nhood',[51 31],'threshold',10);    %101，101
x = T(PP(:,2)); y = R(PP(:,1));
plot(x,y,'x','LineWidth',2,'Color','g');

lines = houghlines(BW,T,R,PP,'FillGap',200,'MinLength',300);


figure, imshow(P), hold on
max_len = 0;

for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end


%  
%  Bw2 = imclose(I, sel);
