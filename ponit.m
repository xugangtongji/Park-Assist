P=imread('2.png');
px1=[0 0];
px2=[0 0];
py1=[0 0];
py2=[0 0];
for i=21:460 
    for j=21:620 
       if P(i,j)>210&&(P(i,j)-P(i,j-2))>100&&(mean(P(i,j:(j+20))>210))
           plot(i,j);
       elseif P(i,j)>210&&(P(i,j+2)-P(i,j))>100&&mean(P(i,(j-20):j))>210
           px2=[i,j];
       end
    end
end