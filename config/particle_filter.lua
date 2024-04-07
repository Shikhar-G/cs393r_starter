function Vector2(x, y)
    return {x = x, y = y}
  end

map = "GDC1";
init_x = 14.7;
init_y = 14.24;
init_r = 0; -- in degrees

k_x = 0.3;
k_y = 0.1;
k_theta = 1.2;
num_particles = 20;
ds_factor = 10;

k_laser_loc=Vector2(0.2, 0.0);

d_short = 1.0; -- 0.2
d_long = 2.0;
likelihood_sigma = 3.0;
likelihood_gamma = 0.8;

dist_threshold = 3.0; -- meters
angle_threshold = 1.0; -- in radians
