pose_dist = 2;
%pose_theta
thetas=[-pi:.1:pi];

vals_size = size(thetas);
num_vals = vals_size(2);

dir_vals=zeros(vals_size);
dir_sigs=zeros(vals_size);

for ind=1:size(thetas,2)
    theta = thetas(ind);
    dir_val = (pose_dist *cos(theta) +0*sin(theta));
    dir_val = dir_val * 100;
    dir_vals(ind) = dir_val;
    
    dir_sig = dir_val / (1 + abs(dir_val));
    dir_sigs(ind) = dir_sig;
end


figure()
%plot(thetas, dir_vals, '-r', thetas, dir_sigs, '-b')
plot(thetas, dir_sigs, '-b')