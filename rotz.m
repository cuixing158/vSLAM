function R = rotz(deg)
R = [cosd(deg),-sind(deg),0;
    sind(deg),cosd(deg),0;
    0,0,1];
end