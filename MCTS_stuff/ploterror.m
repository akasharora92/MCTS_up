figure;
X = 1:50;
scatter(X,bestutility_vect);
errorbar(X,bestutility_vect, stdutility_vect);
hold on;

scatter(X, actualinfogain_vect);