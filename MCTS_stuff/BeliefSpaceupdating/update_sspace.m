function BeliefMaps = update_sspace(obs_vect, DomainKnowledge, BeliefMaps, MapParameters)
%calculates the distribution of silica content based on location estimates

loc_x = obs_vect(5);
loc_y = obs_vect(6);

p_sgivenl = DomainKnowledge.theta_sl;

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
        
        p_lgivenz = BeliefMaps.Location{new_locx, new_locy};
        p_sgivenz = p_sgivenl*p_lgivenz';

        BeliefMaps.Silica{new_locx, new_locy} = p_sgivenz';
    end
end
end

