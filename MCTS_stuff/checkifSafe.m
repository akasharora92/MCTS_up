function [ output ] = checkifSafe( loc_x, loc_y, MapParameters )
%returns 0 if indices are out of map

%future work- add all inaccessible/dangerous cells on the map

if (loc_x < 1 || loc_y < 1)
    output = 0;
elseif ((loc_x > MapParameters.l_rows) || (loc_y > MapParameters.l_cols))
    output = 0;
else
    output = 1;
end


end

