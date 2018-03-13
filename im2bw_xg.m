%二值化识别停车位

im=imread('1.jpg');
%im=rgb2gray(im);
%imhist(im);
figure;imshow(im);

im=imadjust(im,[],[],1.5);
figure;imhist(im);
figure;imshow(im);


bw=im2bw(im,0.5);
imshow(bw);

bw1= bwareaopen(bw,1000);
figure;imshow(bw1);

se=strel('square',8);
bw2= imclose(bw1,se); 
figure;imshow(bw2);

se=strel('square',4);
bw3= imopen(bw2,se); 
figure;imshow(bw3);