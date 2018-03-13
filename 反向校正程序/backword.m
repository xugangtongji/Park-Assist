%反向模型,根号近似效果一般除法模型
clear all;
clc;
img = imread('a (5).jpg');
%img = imread('C:\Users\Administrator\Desktop\一次延拓.JPG');
%imtool(img);
imtool(img); %show this picture
[m,n,cc] = size(img);%得到rgb图片的三个参数,长宽尺寸,与表征rgb的cc值3
mm = m;              %576  720   3
nn = n;              %
C = zeros(mm,nn,cc); %与原图像同样大小但没有像素值的0矩阵
i0 = 240;            %图像中心坐标(293,359)
j0 = 360;               %267  362
di = 500;            %每米象素的个数242 245
dj = 500;
lamda = 0.55;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
for i=1:mm
    for j=1:nn
        x = (i-i0)/di;%(293,359)是原始坐标
        y = (j-j0)/dj;%x,y表示图像坐标
        r=x^2+y^2;
        f=(sqrt(1+4*lamda*r)-1)/(2*lamda*r);%
        a=floor(i0+(i-i0)*f);%a,b为修正后像素坐标 取整round fix ceil floor
        b=floor(j0+(j-j0)*f); 
        if a>0 && b>0 && a <= m && b<=n
        C(i,j,:)=img(a,b,:);%像素填充
        end
    end
end
toc
C = uint8(C);%将C转为uint8型,转化为图像RGB
%figure ;imshow(C);
imtool(C);
imwrite(C,'ax_6.jpg');
