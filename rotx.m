function R = rotx(deg)
R = [1,0,0;
    0,cosd(deg),-sind(deg);
    0,sind(deg),cosd(deg)];
end