function BeliefMaps = update_ALL_s(obs_vect, DomainKnowledge, BeliefMaps, MapParameters, robot)
%updates the belief on silica distributions across the map based on a
%silica measurement

%P(S2|S1) = sum P(S2|L2,S1).P(L2|S1)
%P(L2|S1) was calculates in the earlier step

obs_sil = obs_vect(1);
loc_x = obs_vect(5);
loc_y = obs_vect(6);

p_sgivenl = DomainKnowledge.theta_sl;




%search neighbourhood
search_radius = 80;


location_obs = [loc_x, loc_y];

for i=1:MapParameters.l_rows,
    for j=1:MapParameters.l_cols,
        %get location of the cell
        new_locx = i;
        new_locy = j;
        
        
        %get weighting/correlation
        location_2 = [new_locx, new_locy];
        
        %check if location is outside the search_radius (cell too far to
        %be influenced by observations
        if ((norm(location_2 - location_obs)) > (search_radius/20)),
            continue
        end
        
        
        p_lgivenz = BeliefMaps.Location{i, j};
        
        %update silica
        %         if (norm(location_2 - location_obs) == 0) %fix the belief of the observation taken
        %             belief = zeros(1,3);
        %             belief(obs_sil) = 1;
        %             BeliefMaps.Silica{i, j} = belief;
        %         elseif robot.visibility_silica(i,j) == 0
        %             p_sgivenz = p_sgivenl*p_lgivenz';
        %             BeliefMaps.Silica{new_locx, new_locy} = p_sgivenz';
        %         end
        
        
        p_sgivenz = p_sgivenl*p_lgivenz';
        BeliefMaps.Silica{new_locx, new_locy} = p_sgivenz';
        
        
        p_rgivenz = DomainKnowledge.theta_rl*p_lgivenz';
        p_f1givenz  = DomainKnowledge.theta_fr_1*p_rgivenz;
        p_f2givenz = DomainKnowledge.theta_fr_2*p_rgivenz;
        p_f3givenz  = DomainKnowledge.theta_fr_3*p_rgivenz;
        
        for obs_x = -9:10,
            for obs_y = -9:10,
                
                loc_obs = [new_locx + obs_x, new_locy + obs_y];
                
                %check if values are out of bounds
                output = checkRange(loc_obs(1), loc_obs(2), MapParameters);
                if output == 0,
                    continue
                end
                
                if (robot.visibility(loc_obs(1), loc_obs(2)) ~= 0), %check visibility. Not updating cells where direct observations have been made- ASSUMPTION!!
                    continue
                end
                
                BeliefMaps.Rock{loc_obs(1), loc_obs(2)} = p_rgivenz';
                BeliefMaps.F1{loc_obs(1), loc_obs(2)} = p_f1givenz';
                BeliefMaps.F2{loc_obs(1), loc_obs(2)} = p_f2givenz';
                BeliefMaps.F3{loc_obs(1), loc_obs(2)} = p_f3givenz';
                
            end
        end
    end
end

end





