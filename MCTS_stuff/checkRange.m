function [ output ] = checkRange( loc_x, loc_y, MapParameters )
%returns 0 if indices are out of map

if (loc_x < 1 || loc_y < 1)
    output = 0;
elseif ((loc_x > MapParameters.xsize) || (loc_y > MapParameters.ysize))
    output = 0;
else
    output = 1;
end


end

