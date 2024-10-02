function R = roty(deg)
R = [cosd(deg),0,sind(deg);
    0,1,0;
   -sind(deg),0,cosd(deg)];
end