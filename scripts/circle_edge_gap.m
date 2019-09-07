n=1;

radius = 1;
vals = 0:.01:radius*1.1;

figure(n); n=n+1;

d = radius - vals;
dv = boundFromBelow(d, 0, 0);
plot(d, dv);

figure(n); n=n+1;
plot(d, getCostP(d,1,0));


figure(n); n=n+1;
delta = radius * .3;
offset = 0;
plot(d, getCost(d,0,delta, offset));

figure(n); n=n+1;
delta = radius * .3;
offset = .1;
plot(d, getCost(d,0,delta, offset));

figure(n); n=n+1;
delta = radius * .5;
offset = 0;
plot(d, getCost(d,0,delta, offset));

figure(n); n=n+1;
delta = radius * .5;
offset = .1;
plot(d, getCost(d,0,delta, offset));

figure(n); n=n+1;
ratio = .5;
offset = .2;
v=boundFromBelow(d/(radius*(1-ratio)), 0, 1+offset/(radius*(1-ratio))).^4;
plot(d, v);

figure(n); n=n+1;
ratio = .5;
offsetr = .4;
v=boundFromBelow(d/(radius*((ratio-offsetr))), 0, 1+offsetr*radius/(radius*(1-(ratio-offsetr))));
plot(d, v);


figure(n); n=n+1;
a = .5;
b = .3;
v=boundFromBelow(((-d/radius)-(1-a))/((1-b)-(1-a)), 0, 0);
plot(d, v);

figure(n); n=n+1;
a = .3;
b = .6;
v = vals/radius * (1/(b-a)) - (a/(b-a));
v = v.^4;
plot(vals, v);

figure(n); n=n+1;
a = .4;
b = .6;
v = boundFromBelow(-vals/radius * (1/(b-a)),0, -a/(b-a));
v = v.^4;
plot(vals, v);

function [p] = getCostP(d, pow, offset)
    b = boundFromBelow(d  , 0, offset);
    a = b;% b./delta;
    if pow == 0
        p = a;
    else
        p = a.^(2*pow);
    end
end

function [p] = getCost(d, pow, delta, offset)
    b = boundFromBelow(d/delta  , 0, 1+offset/delta);
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
