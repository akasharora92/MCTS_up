function BeliefMaps = update_lspace_fast(obs, DomainKnowledge, BeliefMaps, p_zgivenr, MapParameters)
%updates the belief on location as well as the locations cells around the
%observations

%extract observations
obs_f1 = obs(1);
obs_f2 = obs(2);
obs_f3 = obs(3);
loc_x  = obs(5);
loc_y  = obs(6);

%loc_x = 500;
%loc_y = 500;
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

x_range = (loc_x-search_radius):(loc_x+search_radius);
y_range = (loc_y-search_radius):(loc_y+search_radius);
[x_loc, y_loc] = meshgrid(x_range, y_range);

%flip matrices so x is row num and y is col num
x_loc = x_loc';
y_loc = y_loc';



weighting_matrix = getSpatialDependency_mat(location_1, x_loc, y_loc);

p_zgivenl2_1 = weighting_matrix.*p_zgivenl(1) + (1-weighting_matrix).*(1/3);
p_zgivenl2_2 = weighting_matrix.*p_zgivenl(2) + (1-weighting_matrix).*(1/3);
p_zgivenl2_3 = weighting_matrix.*p_zgivenl(3) + (1-weighting_matrix).*(1/3);

%x_loc = reshape(x_loc, size(x_loc,1)*size(x_loc,2), 1);
%y_loc = reshape(x_loc, size(x_loc,1)*size(x_loc,2), 1);

%belief_index = BeliefMaps.Location{x_loc, y_loc};

%posterior_L = prior_L.*p_zgivenl2;

%posterior_L =  ((1/sum(posterior_L)).*posterior_L)';

for i=1:length(x_range)
    for j=1:length(y_range), 
        if (i == 0 && j == 0)
            continue
        end
        
        new_locx = x_loc(i,j);
        new_locy = y_loc(i,j);
        
        %check if values are out of bounds
        output = checkRange(new_locx, new_locy, MapParameters);
        if output == 0,
            continue
        end


 
        prior_l2 = BeliefMaps.Location{new_locx, new_locy}';
        
        p_zgivenl2 = [p_zgivenl2_1(i,j); p_zgivenl2_2(i,j); p_zgivenl2_3(i,j)];
        posterior_l2 = prior_l2.*p_zgivenl2;
        BeliefMaps.Location{new_locx, new_locy} = ((1/sum(posterior_l2)).*posterior_l2)';
        
        
    end
end

end


