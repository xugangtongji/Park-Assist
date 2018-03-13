function rectp = rect_xg(I) 

clc;
I = double(I);

%I,R,f,c,k,KK_new
alpha_c=0;


fc=[437.269471478186 439.131625354024];
cc=[358.981914328806 282.175619519936];
kc=[-0.417725052996651 0.181203112776270 0 0,0];
KK = [fc(1) alpha_c*fc(1) cc(1);0 fc(2) cc(2) ; 0 0 1];
R=eye(3);

tic

if size(I,3)>1,
      I = 0.299 * I(:,:,1) + 0.5870 * I(:,:,2) + 0.114 * I(:,:,3);
                                                                %这个系数是什么？ 去掉后就很奇怪
end;    

%[I2] = rect(I,eye(3),fc,cc,kc,KK);


[nr,nc] = size(I);

Irec = 255*ones(nr,nc);

[mx,my] = meshgrid(1:nc, 1:nr);
px = reshape(mx',nc*nr,1);
py = reshape(my',nc*nr,1);

rays = inv(KK)*[(px - 1)';(py - 1)';ones(1,length(px))];

rays2 = R'*rays;

x = [rays2(1,:)./rays2(3,:);rays2(2,:)./rays2(3,:)];


% Add distortion:
%xd = apply_distortion(x,kc);

% Complete the distortion vector if you are using the simple distortion model:
length_k = length(kc);
if length_k <5 ,
    kc = [kc ; zeros(5-length_k,1)];
end;


% Add distortion:

r2 = x(1,:).^2 + x(2,:).^2;

r4 = r2.^2;

r6 = r2.^3;


% Radial distortion: 径向矫正

cdist = 1 + kc(1) * r2 + kc(2) * r4 + kc(5) * r6;


xd1 = x .* (ones(2,1)*cdist);


% tangential distortion:切向矫正

a1 = 2.*x(1,:).*x(2,:);
a2 = r2 + 2*x(1,:).^2;
a3 = r2 + 2*x(2,:).^2;

delta_x = [kc(3)*a1 + kc(4)*a2 ;
   kc(3) * a3 + kc(4)*a1];



xd = xd1 + delta_x;






%%%%%%%%%%%%%%%%%%%%%
px2 = fc(1)*(xd(1,:)+alpha_c*xd(2,:))+cc(1);
py2 = fc(2)*xd(2,:)+cc(2);

px_0 = floor(px2);


py_0 = floor(py2);
py_1 = py_0 + 1;

good_points = find((px_0 >= 0) & (px_0 <= (nc-2)) & (py_0 >= 0) & (py_0 <= (nr-2)));

px2 = px2(good_points);
py2 = py2(good_points);
px_0 = px_0(good_points);
py_0 = py_0(good_points);

alpha_x = px2 - px_0;
alpha_y = py2 - py_0;

a1 = (1 - alpha_y).*(1 - alpha_x);
a2 = (1 - alpha_y).*alpha_x;
a3 = alpha_y .* (1 - alpha_x);
a4 = alpha_y .* alpha_x;

ind_lu = px_0 * nr + py_0 + 1;
ind_ru = (px_0 + 1) * nr + py_0 + 1;
ind_ld = px_0 * nr + (py_0 + 1) + 1;
ind_rd = (px_0 + 1) * nr + (py_0 + 1) + 1;

ind_new = (px(good_points)-1)*nr + py(good_points);


Irec(ind_new) = a1.* I(ind_lu) + a2.* I(ind_ru) + a3.* I(ind_ld) + a4.* I(ind_rd);

%imtool(Irec);
toc
rectp=uint8(round(Irec));
%imtool(rectp);

%imwrite(uint8(round(Irec)),gray(256),'kk.bmp')

%return;  rect.m





