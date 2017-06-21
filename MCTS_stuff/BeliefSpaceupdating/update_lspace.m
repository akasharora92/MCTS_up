function [BeliefMaps, infoGain] = update_lspace(obs, DomainKnowledge, BeliefMaps, p_zgivenr, MapParameters)
%updates the belief on location as well as the locations cells around the
%observations

%extract observations
obs_f1 = obs(1);
obs_f2 = obs(2);
obs_f3 = obs(3);
loc_x  = obs(5);
loc_y  = obs(6);

%deterministic parameters
p_rl = DomainKnowledge.theta_rl';
p_zgivenl = p_rl*p_zgivenr';
p_zgivenl = (1/sum(p_zgivenl)).*p_zgivenl;


%uncertain parameters- requires a distribution for each parameter
%assume independence
%can also enforce the contraint that each column must add up to 1. 
%need integration/sampling

%spatial updating
location_obs = [loc_x, loc_y];

%search neighbourhood
search_radius = 80;

infoGain = 0;
%infoGain_map = zeros(search_radius);

for i=1:MapParameters.l_rows,
    for j=1:MapParameters.l_cols, 
        
        %get location of the centre of the cell in terms of rock grid
        new_locx = i*20-10;
        new_locy = j*20-10;
        
        
        %get weighting/correlation
        location_2 = [new_locx, new_locy];
        
        %check if location is outside the search_radius (cell too far to
        %have much influence on observations
        if ((norm(location_2 - location_obs)) > search_radius),
            continue
        end
        
        weighting = getSpatialDependency(location_obs, location_2);
        
        p_zgivenl2 = weighting.*p_zgivenl + (1-weighting).*[1/3; 1/3; 1/3];
                
        prior_l2 = BeliefMaps.Location{i,j}';
        
        prior_ent = abs(sum(prior_l2.*log(prior_l2)));
        
        posterior_l2 = prior_l2.*p_zgivenl2;
        posterior_l2 = ((1/sum(posterior_l2)).*posterior_l2)';
        
        posterior_ent = abs(sum(posterior_l2.*log(posterior_l2)));
        BeliefMaps.Location{i,j} = posterior_l2;
         
        infoGain = (prior_ent - posterior_ent) + infoGain;
        
    end
end

%figure; imagesc(infoGain_map);
end

