function BeliefMaps = update_rspace_s(obs_vect, DomainKnowledge, BeliefMaps, MapParameters, robot)
%update belief on rocks given measurement s. 

loc_x = obs_vect(5);
loc_y = obs_vect(6);

p_rgivenl = DomainKnowledge.theta_rl;

%search neighbourhood
search_radius = 10;

for i=-search_radius:search_radius,
    for j=-search_radius:search_radius, 

        new_locx = loc_x + i;
        new_locy = loc_y + j;
        
        %check if values are out of bounds
        output = checkRange(new_locx, new_locy, MapParameters);
        if output == 0,
            continue
        end
        
        %check if cell has been seen before..
        if robot.visibility(new_locx, new_locy) == 1,
            continue
        end
        
        p_lgivens = BeliefMaps.Location{new_locx, new_locy};
        p_rgivens = p_rgivenl*p_lgivens';

        BeliefMaps.Rock{new_locx, new_locy} = p_rgivens';
    end
end

end

