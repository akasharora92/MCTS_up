function [BeliefMaps, p_zgivenr, new_msg] = update_rspace(obs, DomainKnowledge, BeliefMaps, sensor, previous_obs, old_msg)
%updates the posterior distribution of rock types
%P(R|Z1, Z2, Z3) = n*P(Z1, Z2, Z3|R)*P(R)
%                = n*P(Z1|R)*P(Z2|R)*P(Z3|R)*P(R)

obs_f1 = obs(1);
obs_f2 = obs(2);
obs_f3 = obs(3);
loc_x  = obs(5);
loc_y  = obs(6);

%rock prior
rock_prior = BeliefMaps.Rock{loc_x, loc_y};

%if no previous observations have been taken on the rock..
if isempty(previous_obs)
    %probability of Z1|R
    p_zf1 = sensor.noise_f1(obs_f1,:);
    p_f1r = DomainKnowledge.theta_fr_1;
    p_z1r = p_zf1*p_f1r;
    
    %probability of Z2|R
    p_zf2 = sensor.noise_f1(obs_f2,:);
    p_f2r = DomainKnowledge.theta_fr_2;
    p_z2r = p_zf2*p_f2r;
    
    %probability of Z3|R
    p_zf3 = sensor.noise_f1(obs_f3,:);
    p_f3r = DomainKnowledge.theta_fr_3;
    p_z3r = p_zf3*p_f3r;
    
    %probability of Z|R
    p_zgivenr = p_z1r.*p_z2r.*p_z3r;
    new_msg = p_zgivenr;
    
else
    %if previous observations have been taken on the rock..
    
    %probability of Z1|F
    p_zf1 = sensor.noise_f1(obs_f1,:);
    p_zf2 = sensor.noise_f1(obs_f2,:);
    p_zf3 = sensor.noise_f1(obs_f3,:);
    
    %for each set of previous observations
    for i=1:size(previous_obs,1)
        p_zf1 = p_zf1.*sensor.noise_f1(previous_obs(i,1),:);
        p_zf2 = p_zf2.*sensor.noise_f1(previous_obs(i,2),:);
        p_zf3 = p_zf3.*sensor.noise_f1(previous_obs(i,3),:);
    end
    
    %get classifier likelihoods
    p_f1r = DomainKnowledge.theta_fr_1;
    p_f2r = DomainKnowledge.theta_fr_2;
    p_f3r = DomainKnowledge.theta_fr_3;
    
    p_z1r = p_zf1*p_f1r;
    p_z2r = p_zf2*p_f2r;
    p_z3r = p_zf3*p_f3r;
    
    %probability of Z|R, previousobs
    new_msg = (p_z1r.*p_z2r.*p_z3r);
    p_zgivenr = new_msg./old_msg;
    
    
end


p_rgivenz = p_zgivenr.*rock_prior;
p_rgivenz = (1/sum(p_rgivenz)).*p_rgivenz;

BeliefMaps.Rock{loc_x, loc_y} = p_rgivenz;


end

