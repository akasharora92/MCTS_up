function  [BeliefMaps] = updateALL(obs_vect, BeliefMaps, DomainKnowledge, MapParameters, robot)
%updates all the node beliefs- useful for visualisation

loc_x = obs_vect(5);
loc_y = obs_vect(6);

p_sgivenl = DomainKnowledge.theta_sl;

%search neighbourhood
search_radius = 100;

%can be sped up. we are redoing a lot of calculation

% for i=-search_radius:search_radius,
%     for j=-search_radius:search_radius,
%
%         new_locx = loc_x + i;
%         new_locy = loc_y + j;
%
%         %check if values are out of bounds
%         output = checkRange(new_locx, new_locy, MapParameters);
%         if output == 0,
%             continue
%         end
%
%         if (i == 0 && j == 0),
%             continue; %no need to update rocks and features as we already calculated this earlier
%         elseif (robot.visibility(new_locx, new_locy) ~= 0), %check visibility. Not updating cells where direct observations have been made- ASSUMPTION!!
%             continue
%         end
%
%         %find the location cell you are in
%         cell_loc_x = round(new_locx/20);
%         cell_loc_y = round(new_locy/20);
%
%         if cell_loc_x == 0,
%             cell_loc_x = 1;
%         end
%
%         if cell_loc_y == 0,
%             cell_loc_y = 1;
%         end
%
%         %update silica
%         p_lgivenz = BeliefMaps.Location{cell_loc_x, cell_loc_y};
%         p_sgivenz = p_sgivenl*p_lgivenz';
%         BeliefMaps.Silica{cell_loc_x, cell_loc_y} = p_sgivenz';
%
%         p_rgivenz = DomainKnowledge.theta_rl*p_lgivenz';
%         p_f1givenz  = DomainKnowledge.theta_fr_1*p_rgivenz;
%         p_f2givenz = DomainKnowledge.theta_fr_2*p_rgivenz;
%         p_f3givenz  = DomainKnowledge.theta_fr_3*p_rgivenz;
%
%         BeliefMaps.Rock{new_locx, new_locy} = p_rgivenz';
%         BeliefMaps.F1{new_locx, new_locy} = p_f1givenz';
%         BeliefMaps.F2{new_locx, new_locy} = p_f2givenz';
%         BeliefMaps.F3{new_locx, new_locy} = p_f3givenz';
%
%     end
% end

location_obs = [loc_x, loc_y];

for i=1:MapParameters.l_rows,
    for j=1:MapParameters.l_cols,
        %get location of the cell
        new_locx = i*20-10;
        new_locy = j*20-10;
        
        
        %get weighting/correlation
        location_2 = [new_locx, new_locy];
        
        %check if location is outside the search_radius (cell too far to
        %be influenced by observations
        if ((norm(location_2 - location_obs)) > search_radius),
            continue
        end
        
        %update silica
        p_lgivenz = BeliefMaps.Location{i, j};
        p_sgivenz = p_sgivenl*p_lgivenz';
        BeliefMaps.Silica{i,j} = p_sgivenz';
        
        p_rgivenz = DomainKnowledge.theta_rl*p_lgivenz';
        p_f1givenz  = DomainKnowledge.theta_fr_1*p_rgivenz;
        p_f2givenz = DomainKnowledge.theta_fr_2*p_rgivenz;
        p_f3givenz  = DomainKnowledge.theta_fr_3*p_rgivenz;
        
        %update rock grid
        for obs_x = -9:10,
            for obs_y = -9:10,
                
                loc_obs = [new_locx + obs_x, new_locy + obs_y];
                
                %check if values are out of bounds
                
                if (loc_obs(1) < 1 || loc_obs(2) < 1)
                    output = 0;
                elseif ((loc_obs(1) > MapParameters.xsize) || (loc_obs(2) > MapParameters.ysize))
                    output = 0;
                else
                    output = 1;
                end
                
                %output = checkRange(loc_obs(1), loc_obs(2), MapParameters);
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






