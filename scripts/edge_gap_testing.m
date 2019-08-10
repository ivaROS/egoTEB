vals = -2:.01:2;

figure(1);
bounded = boundFromBelow(vals, .5, 0);
plot(vals, bounded);
bounded = boundFromBelow(vals, .5, .1);
hold on
plot(vals, bounded);
hold off


dists = -1:.01:1;

figure(2)
costs = getCost(dists, 1, 0);
plot(dists, costs, '-b')
costs = getCost(dists, 2, 0);
hold on
plot(dists, costs, '-r')
hold off


figure(3)
costs = getCost(dists, 1, .1);
plot(dists, costs, '-b')
costs = getCost(dists, 1, .2);
hold on
plot(dists, costs, '-r')
hold off


figure(4)
gap_left = [4,2];
gap_right = [5,-1];

pose = [3, -.25];

poses_y = -1:.05:2;
poses = zeros(2, size(poses_y,2));
poses(2,:) = poses_y;
poses(1,:) = 3;

plot(gap_left(1), gap_left(2), '*r')
hold on
plot(gap_right(1), gap_right(2), '*r')

line([0, gap_left(1)],[0, gap_left(2)])
line([0, gap_right(1)],[0, gap_right(2)])

left_norm = getLeftBorderNormal(gap_left);
right_norm = getRightBorderNormal(gap_right);

line([0, left_norm(1)],[0, left_norm(2)], 'Color','green','LineStyle','--')
line([0, right_norm(1)],[0, right_norm(2)], 'Color','green','LineStyle','--')

plot(poses(1,:), poses(2,:), '*k')

ldot = dot(pose, left_norm);
rdot = dot(pose, right_norm);

ldot = left_norm * poses;
rdot = right_norm * poses;

epsilon = .1;
lcost = getCost(ldot, 1, epsilon);
rcost = getCost(rdot, 1, epsilon);

XL = get(gca, 'XLim')
YL = get(gca, 'YLim')

%annotation('arrow', [0, .2], [0, 0]);
axis equal
hold off


figure(5)
plot(poses(2,:), lcost, '--g')
hold on
plot(poses(2,:), rcost, '--r')

figure(6)
xlim(XL)
ylim(YL)

resolution = .1;

xset = min(XL):resolution:max(XL);
yset = min(YL):resolution:max(YL);
[xg,yg] = meshgrid(xset,yset);

poses = [reshape(xg, 1, []); reshape(yg, 1, [])];
costs = posesToCost(poses, gap_left, gap_right, .1, 2);

shapedcosts = reshape(costs, size(xg));

colormap('hsv')
%set(gca,'YDir','normal')

imagesc('XData', poses(1,:), 'YData', poses(2,:), 'CData', shapedcosts)
%imagesc(flipud(poses(1,:)), flipud(poses(2,:)), flipud(costs))
colorbar

figure(7)
surf(xg, yg, shapedcosts, 'FaceAlpha', .2, 'FaceColor', 'interp')


function [v] = posesToCost(poses, gap_left, gap_right, epsilon, power)

    left_norm = getLeftBorderNormal(gap_left);
    right_norm = getRightBorderNormal(gap_right);

    ldot = left_norm * poses;
    rdot = right_norm * poses;

    lcost = getCost(ldot, power, epsilon);
    rcost = getCost(rdot, power, epsilon);
    v = max(lcost, rcost);
end

function [v] = getLeftBorderNormal(left_edge)
    v=zeros(size(left_edge));
    v(1) = -left_edge(2);
    v(2) = left_edge(1);
    v = v/(norm(v));
end

function [v] = getRightBorderNormal(right_edge)
    v=zeros(size(right_edge));
    v(1) = right_edge(2);
    v(2) = -right_edge(1);
    v = v/(norm(v));
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
