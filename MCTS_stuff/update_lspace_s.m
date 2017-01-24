function [BeliefMaps, infoGain] = update_lspace_s(obs_vect, DomainKnowledge, BeliefMaps, MapParameters)
%updates the belief on location as well as the locations cells around the
%observations

%extract observations
obs_silica = obs_vect(1);
loc_x = obs_vect(5);
loc_y = obs_vect(6);

%extract relevant conditional probability parameters
p_sgivenl = DomainKnowledge.theta_sl(obs_silica, :);
%prior_L = BeliefMaps.Location{loc_x, loc_y}';
%posterior_L = p_sgivenl'.*prior_L;

%BeliefMaps.Location{loc_x, loc_y} = ((1/sum(posterior_L)).*posterior_L)';


%spatial updating
location_obs = [loc_x, loc_y];

%search neighbourhood
search_radius = 80;

infoGain = 0;

for i=1:MapParameters.l_rows,
    for j=1:MapParameters.l_cols,       
        
        %get weighting/correlation
        location_2 = [i,j];
        
        %check if location is outside the search_radius (cell too far to
        %have much influence on observations
        if ((20*norm(location_2 - location_obs)) > search_radius),
            continue
        end
        
        weighting = getSpatialDependency(20*location_obs, 20*location_2);
        p_sgivenl2 = weighting.*p_sgivenl' + (1-weighting).*[1/3; 1/3; 1/3];
        
        prior_l2 = BeliefMaps.Location{i,j}';
        prior_ent = abs(sum(prior_l2.*log(prior_l2)));
        
        posterior_l2 = prior_l2.*p_sgivenl2;
        
        posterior_l2 =((1/sum(posterior_l2)).*posterior_l2);
        
        posterior_ent = abs(sum(posterior_l2.*log(posterior_l2)));
        
        BeliefMaps.Location{i,j} = posterior_l2';
        
        infoGain = (prior_ent - posterior_ent) + infoGain;
        
    end
end

end


