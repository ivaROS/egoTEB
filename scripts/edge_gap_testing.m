n=1;

vals = -1:.01:1;

figure(n); n=n+1;
bounded = boundFromBelow(vals, .5, 0);
plot(vals, bounded);
bounded = boundFromBelow(vals, .5, .1);
hold on
plot(vals, bounded);
title('bounding function: epsilon')
legend('epsilon=0', 'epsilon=.1')
hold off


dists = -0.15:.01:.1;
figure(n); n=n+1;
costs = getCost(dists, 0, .1, 0);
plot(dists, costs, '-b')
costs = getCost(dists, 1, .1, 0);
hold on
plot(dists, costs, '-r')
costs = getCost(dists, 2, .1, 0);
plot(dists, costs, '-g')
title('cost function: power (delta=.1)')
legend('power=0','power=2', 'power=4')
hold off

dists = -0.25:.01:.25;
figure(n); n=n+1;
costs = getCost(dists, 1, .1, 0);
plot(dists, costs, '-b')
costs = getCost(dists, 1, .2, 0);
hold on
plot(dists, costs, '-r')
title('cost function: delta')
legend('delta=.1', 'delta=.2')
hold off

dists = -0.35:.01:.55;
figure(n); n=n+1;
costs = getCost(dists, 1, .1, 0);
plot(dists, costs, '-b')
costs = getCost(dists, 1, .1, .125);
hold on
plot(dists, costs, '-r')
costs = getCost(dists, 1, .1, .25);
plot(dists, costs, '-g')
title('cost function: offset')
legend('offset=0', 'offset=.1', 'offset=.25')
hold off


figure(n); n=n+1;
gap_left = [4,2];
gap_right = [5,-1];

x0 = [0.5,1.4];

pose = [3, -.25];

poses_y = -1:.05:2;
poses = zeros(2, size(poses_y,2));
poses(2,:) = poses_y;
poses(1,:) = 3;

plot(gap_left(1), gap_left(2), '*r')
hold on
plot(gap_right(1), gap_right(2), '*r')

line([x0(1), gap_left(1)],[x0(2), gap_left(2)])
line([x0(1), gap_right(1)],[x0(2), gap_right(2)])

left_norm = getLeftBorderNormal(gap_left-x0);
right_norm = getRightBorderNormal(gap_right-x0);

line([x0(1), left_norm(1)+x0(1)],[x0(2), left_norm(2)+x0(2)], 'Color','green','LineStyle','--')
line([x0(1), right_norm(1)+x0(1)],[x0(2), right_norm(2)+x0(2)], 'Color','green','LineStyle','--')

plot(poses(1,:), poses(2,:), '*k')

ldot = dot(pose, left_norm);
rdot = dot(pose, right_norm);

ldot = left_norm * (poses-x0');
rdot = right_norm * (poses-x0');

delta = .1;
offset = .1;
lcost = getCost(ldot, 1, delta, offset);
rcost = getCost(rdot, 1, delta, offset);

XL = get(gca, 'XLim');
YL = get(gca, 'YLim');

%annotation('arrow', [0, .2], [0, 0]);
axis equal
hold off


figure(n); n=n+1;
plot(poses(2,:), lcost, '--g')
hold on
plot(poses(2,:), rcost, '--r')
hold off


figure(n); n=n+1;
xlim(XL)
ylim(YL)

resolution = .1;

xset = min(XL):resolution:max(XL);
yset = min(YL):resolution:max(YL);
[xg,yg] = meshgrid(xset,yset);

poses = [reshape(xg, 1, []); reshape(yg, 1, [])];

pow=1;
delta=.1;
offset=.1;
costs = posesToCost(poses, gap_left, gap_right, pow, delta, offset);

shapedcosts = reshape(costs, size(xg));

%colormap('hsv')
%set(gca,'YDir','normal')

myColors = parula(256);
cLim = [min(costs), max(costs)];
centerPoint = 1;
scalingIntensity = 1;
inc = 1;
[newMap, ticks, tickLabels] = MC_nonlinearCmap(myColors, centerPoint, cLim, scalingIntensity, inc);

colormap(newMap)
imagesc('XData', poses(1,:), 'YData', poses(2,:), 'CData', shapedcosts);
hold on
%imagesc(flipud(poses(1,:)), flipud(poses(2,:)), flipud(costs))
colorbar
hold off

figure(n); n=n+1;
surf(xg, yg, shapedcosts, 'FaceAlpha', .2, 'FaceColor', 'interp')
hold off

function [v] = posesToCost(poses, gap_left, gap_right, pow, delta, offset)
    left_norm = getLeftBorderNormal(gap_left);
    right_norm = getRightBorderNormal(gap_right);

    ldot = left_norm * poses;
    rdot = right_norm * poses;

    lcost = getCost(ldot, pow, delta, offset);
    rcost = getCost(rdot, pow, delta, offset);
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

function [p] = getCost(dotprod, pow, delta, offset)
    b = boundFromBelow(-dotprod/delta  , 0, 1+offset/delta);
    a = b;% b./delta;
    if pow == 0
        p = a;
    else
        p = a.^(2*pow);
    end
end

function [r] = boundFromBelow(x, lowerbound, epsilon)
    r = zeros(size(x));
    r(x>=lowerbound+epsilon)=0;
    r(x<lowerbound+epsilon)=-x(x<lowerbound+epsilon)+lowerbound+epsilon;
end

function [newMap, ticks, tickLabels] = MC_nonlinearCmap(myColors, centerPoint, cLim, scalingIntensity, inc)
    dataMax = cLim(2);
    dataMin = cLim(1);
    nColors = size(myColors,1);
    colorIdx = 1:size(myColors,1);
    colorIdx = colorIdx - (centerPoint-dataMin)*numel(colorIdx)/(dataMax-dataMin); % idx wrt center point
    colorIdx = scalingIntensity * colorIdx/max(abs(colorIdx));  % scale the range
    colorIdx = sign(colorIdx).*colorIdx.^2;
    colorIdx = colorIdx - min(colorIdx);
    colorIdx = colorIdx*nColors/max(colorIdx)+1;
    newMap = interp1(colorIdx, myColors, 1:nColors);
      if nargout > 1
          % ticks and tickLabels will mark [centerPoint-inc, ... centerPoint+inc, centerPoint+2*inc]
          % on a linear color bar with respect the colors corresponding to the new non-linear colormap
          linear_cValues = linspace(cLim(1), cLim(2), nColors);
          nonlinear_cValues = interp1(1:nColors, linear_cValues, colorIdx);
          tickVals = fliplr(centerPoint:-inc:cLim(1));
          tickVals = [tickVals(1:end-1), centerPoint:inc:cLim(2)];
          ticks = nan(size(tickVals));
          % find what linear_cValues correspond to when nonlinear_cValues==ticks
          for i = 1:numel(tickVals)
              [~, idx] = min(abs(nonlinear_cValues - tickVals(i)));
              ticks(i) = linear_cValues(idx);
          end
          tickLabels = arrayfun(@num2str, tickVals, 'Uniformoutput', false);
      end
  end