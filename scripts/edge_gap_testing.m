vals = -2:.01:2;

figure(1);
bounded = boundFromBelow(vals, .5, 0);
plot(vals, bounded);
bounded = boundFromBelow(vals, .5, .1);
hold on
plot(vals, bounded);
hold off


dists = -1:.01:1;

figure(3)
costs = getCost(dists, 1, 0);
plot(dists, costs, '-b')
costs = getCost(dists, 2, 0);
hold on
plot(dists, costs, '-r')
hold off

figure(4)
costs = getCost(dists, 1, .1);
plot(dists, costs, '-b')
costs = getCost(dists, 1, .2);
hold on
plot(dists, costs, '-r')
hold off

figure(5)
gap_left = [4,2];
gap_right = [5,2];

left_norm = getLeftBorderNormal(gap_left)
right_norm = getRightBorderNormal(gap_right)


function [v] = getLeftBorderNormal(left_edge)
    v=zeros(size(left_edge))
    v(1) = -left_edge(2)
    v(2) = left_edge(1)
end

function [v] = getRightBorderNormal(right_edge)
    v=zeros(size(right_edge))
    v(1) = right_edge(2)
    v(2) = -right_edge(1)
end

function [p] = getCost(dotprod, power, epsilon)
    b = boundFromBelow(-dotprod, 0, epsilon);
    p = b.^(2*power);

end

function [r] = boundFromBelow(x, lowerbound, epsilon)
    r = zeros(size(x));
    r(x>=lowerbound+epsilon)=0;
    r(x<lowerbound+epsilon)=-x(x<lowerbound+epsilon)+lowerbound+epsilon;
end
