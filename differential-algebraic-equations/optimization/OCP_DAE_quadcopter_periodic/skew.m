function [ R ] = skew( w )
    x = w(1);
    y = w(2);
    z = w(3);
    R = [0,-z,y;z,0,-x;-y,x,0];
end

