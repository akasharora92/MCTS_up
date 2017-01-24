function BeliefMaps = update_fspace_s(obs_vect, DomainKnowledge, BeliefMaps, MapParameters, robot)
%update belief on rocks given measurement s. 

loc_x = obs_vect(5);
loc_y = obs_vect(6);

p_f_1givenr = DomainKnowledge.theta_fr_1;
p_f_2givenr = DomainKnowledge.theta_fr_2;
p_f_3givenr = DomainKnowledge.theta_fr_3;

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
        
        
        
        p_rgivens = BeliefMaps.Rock{new_locx, new_locy};
        p_f1givens = p_f_1givenr*p_rgivens';
        p_f2givens = p_f_2givenr*p_rgivens';
        p_f3givens = p_f_3givenr*p_rgivens';

        BeliefMaps.F1{new_locx, new_locy} = p_f1givens';
        BeliefMaps.F2{new_locx, new_locy} = p_f2givens';
        BeliefMaps.F3{new_locx, new_locy} = p_f3givens';
    end
end

end

