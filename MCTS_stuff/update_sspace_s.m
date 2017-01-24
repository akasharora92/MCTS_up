function BeliefMaps = update_sspace_s(obs_vect, DomainKnowledge, BeliefMaps, MapParameters)
%updates the belief on silica distributions across the map based on a
%silica measurement

%P(S2|S1) = sum P(S2|L2,S1).P(L2|S1)
%P(L2|S1) was calculates in the earlier step

obs_sil = obs_vect(4);
loc_x = obs_vect(5);
loc_y = obs_vect(6);

p_sgivenl = DomainKnowledge.theta_sl;

%search neighbourhood
search_radius = 10;

for i=-search_radius:search_radius,
    for j=-search_radius:search_radius, 
        if (i == 0 && j == 0) %fix the belief of the observation taken
            belief = zeros(1,3);
            belief(obs_sil) = 1;
            BeliefMaps.Silica{loc_x, loc_y} = belief;
            continue
        end
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



