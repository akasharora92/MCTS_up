%%generates initial positions and saves to a vector

position_vect = zeros(50,3);

for i = 1:50,
    xpos = round(rand(1)*(MapParameters.l_rows/2)) + 5; %100 cell padding to prevent rovers starting from the edge
    ypos = round(rand(1)*(MapParameters.l_cols-10)) + 5; 
    orientation = 0;
    
    position_vect(i,:) = [xpos, ypos, orientation];
    
    
end


save('initialPos.mat', 'position_vect');