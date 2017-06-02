function [BeliefMaps, infoGain] = update_lspace_approx(obs, DomainKnowledge, BeliefMaps, p_zgivenr, MapParameters)
%updates the belief on location as well as the locations cells around the
%observations

%extract observations
obs_f1 = obs(1);
obs_f2 = obs(2);
obs_f3 = obs(3);
loc_x  = obs(5);
loc_y  = obs(6);

prior_L = BeliefMaps.Location{loc_x, loc_y}';

%deterministic parameters
p_rl = DomainKnowledge.theta_rl';
p_zgivenl = p_rl*p_zgivenr';

%silica CPT
p_sgivenl = DomainKnowledge.theta_sl;


%uncertain parameters- requires a distribution for each parameter
%assume independence
%can also enforce the contraint that each column must add up to 1. 
%need integration/sampling

posterior_L = prior_L.*p_zgivenl;
posterior_L = (1/sum(posterior_L)).*posterior_L;

BeliefMaps.Location{loc_x, loc_y} = posterior_L';

%spatial updating
location_1 = [loc_x, loc_y];

%search neighbourhood
search_radius = 200;

%high res neighbourhood
search_radius_highres = search_radius/4;

infoGain = 0;
%infoGain_map = zeros(search_radius);

for i=-search_radius_highres:search_radius_highres,
    for j=-search_radius_highres:search_radius_highres, 
        if (i == 0 && j == 0)
            continue
        end
        new_locx = loc_x + i;
        new_locy = loc_y + j;
        
        %check if values are out of bounds
        output = checkRange(new_locx, new_locy, MapParameters);
        if output == 0,
            continue
        end
        
        %get weighting/correlation
        location_2 = [new_locx, new_locy];
        weighting = getSpatialDependency(location_1, location_2);
  
        %update location 2 belief
        %P(L2|Z) = n.P(Z|L2).P(L2)
        %P(Z|L2) = weight*P(Z|L1) + (1-weight)*(1/3 1/3 1/3)
        
        p_zgivenl2 = weighting.*p_zgivenl + (1-weighting).*[1/3; 1/3; 1/3];
        
        
        prior_l2 = BeliefMaps.Location{new_locx, new_locy}';
        
        prior_ent = abs(sum(prior_l2.*log(prior_l2)));
        
        posterior_l2 = prior_l2.*p_zgivenl2;
        posterior_l2 = ((1/sum(posterior_l2)).*posterior_l2)';
        
        posterior_ent = abs(sum(posterior_l2.*log(posterior_l2)));
        BeliefMaps.Location{new_locx, new_locy} = posterior_l2;
        
       % infoGain_map(i+201,j+201) = (prior_ent - posterior_ent);
        
        infoGain = (prior_ent - posterior_ent) + infoGain;
        
%        BeliefMaps = updateALL(BeliefMaps, DomainKnowledge);
        %BeliefMaps.Silica{new_locx, new_locy} = 
        %bayes update the locations..
        %p_l2givenl1 = (1-weighting).*[1/3; 1/3; 1/3] + weighting.*[
        
        
    end
end

%grid map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %           %
   %1   %     2     %    3   
        %           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %           %
    %4  %           %     5
        %           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %           %   
  %6    %     7     %    8
        %           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%calculate middle of the grids as well as size
grid_vect = zeros(8,3);

%low res info gains- defining the grid boundaries
grid_1point = [-search_radius, -search_radius];
grid_1_x = length(-search_radius:-search_radius_highres);
grid_1_y = length(-search_radius:-search_radius_highres);
grid_vect(1,:) = [grid_1point(1) + 0.5*grid_1_x, grid_1point(2) + 0.5*grid_1_y, grid_1_x*grid_1_y];

grid_2point = [-search_radius, -search_radius_highres];
grid_2_x = length(-search_radius:-search_radius_highres);
grid_2_y = length(-search_radius_highres:search_radius_highres);
grid_vect(2,:) = [grid_2point(1) + 0.5*grid_2_x, grid_2point(2) + 0.5*grid_2_y, grid_2_x*grid_2_y];

grid_3point = [-search_radius,search_radius_highres];
grid_3_x = length(-search_radius:-search_radius_highres);
grid_3_y = length(search_radius_highres:search_radius);
grid_vect(3,:) = [grid_3point(1) + 0.5*grid_3_x, grid_3point(2) + 0.5*grid_3_y, grid_3_x*grid_3_y];

grid_4point = [-search_radius_highres, -search_radius];
grid_4_x = length(-search_radius_highres:search_radius_highres);
grid_4_y = length(-search_radius:-search_radius_highres);
grid_vect(4,:) = [grid_4point(1) + 0.5*grid_4_x, grid_4point(2) + 0.5*grid_4_y, grid_4_x*grid_4_y];

grid_5point = [-search_radius_highres, search_radius_highres];
grid_5_x = length(-search_radius_highres:search_radius_highres);
grid_5_y = length(search_radius_highres:search_radius);
grid_vect(5,:) = [grid_5point(1) + 0.5*grid_5_x, grid_5point(2) + 0.5*grid_5_y, grid_5_x*grid_5_y];

grid_6point = [search_radius_highres, -search_radius];
grid_6_x = length(search_radius_highres:search_radius);
grid_6_y = length(-search_radius:-search_radius_highres);
grid_vect(6,:) = [grid_6point(1) + 0.5*grid_6_x, grid_6point(2) + 0.5*grid_6_y, grid_6_x*grid_6_y];

grid_7point = [search_radius_highres, -search_radius_highres];
grid_7_x = length(search_radius_highres:search_radius);
grid_7_y = length(-search_radius_highres:search_radius_highres);
grid_vect(7,:) = [grid_7point(1) + 0.5*grid_7_x, grid_7point(2) + 0.5*grid_7_y, grid_7_x*grid_7_y];

grid_8point = [search_radius_highres, search_radius_highres];
grid_8_x = length(search_radius_highres:search_radius);
grid_8_y = length(search_radius_highres:search_radius);
grid_vect(8,:) = [grid_8point(1) + 0.5*grid_8_x, grid_8point(2) + 0.5*grid_8_y, grid_8_x*grid_8_y];

grid_vect = round(grid_vect);

%evaluate information for each grid cell
for i = 1:size(grid_vect,1)
    new_locx = loc_x + grid_vect(i,1);
    new_locy = loc_y + grid_vect(i,2);
    
%    check if values are out of bounds
    output = checkRange(new_locx, new_locy, MapParameters);
    if output == 0,
        continue
    end
    
%    get weighting/correlation
    location_2 = [new_locx, new_locy];
    weighting = getSpatialDependency(location_1, location_2);
    
    p_zgivenl2 = weighting.*p_zgivenl + (1-weighting).*[1/3; 1/3; 1/3];    
    
    prior_l2 = BeliefMaps.Location{new_locx, new_locy}';
    
    prior_ent = abs(sum(prior_l2.*log(prior_l2)));
    
    posterior_l2 = prior_l2.*p_zgivenl2;
    posterior_l2 = ((1/sum(posterior_l2)).*posterior_l2)';
    
    posterior_ent = abs(sum(posterior_l2.*log(posterior_l2)));
    BeliefMaps.Location{new_locx, new_locy} = posterior_l2;
    
    
    infoGain = grid_vect(i,3)*(prior_ent - posterior_ent) + infoGain;
    
end

end

