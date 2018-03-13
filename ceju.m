%k=[487 135 1];  %[477 146 1] [486 135 1]
I=imread('ceju.png');
rectp = rect_xg(I);
imshow(rectp);axis image
p = ginput(1);
p = [p,1];

H=[ 1.4669    2.9188  319.9262;
   -0.9782   -0.1200  620.6380;
    0.0044   -0.0000    1.0000];
PP=inv(H)*p';                     %算出世界坐标
PP=PP/diag(PP(3,:));                   %统一为[xw,yw,1]
PP(3,:)=[];                                       %清除第三行
PP=PP';
zuobiao=num2str(PP);
juli=num2str(sqrt(PP(1)^2+PP(2)^2));
h = msgbox({'空间坐标：',zuobiao,'距离：',juli});

% [imagePoints, boardSize] = detectCheckerboardPoints(rectp);
% squareSize = 25;  % in units of 'mm'
% worldPoints = generateCheckerboardPoints(boardSize, squareSize);
% worldPoints=worldPoints';
% B=[-100;0];
% worldPoints=bsxfun(@minus, worldPoints, B);
% worldPoints=worldPoints';
% H = fitgeotrans(worldPoints, imagePoints, 'projective'); %P
% H = (H.T)';
% H = H / H(3,3);
