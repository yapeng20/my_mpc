% addpath('E:\组内资料\MPC\casadi\casadi-windows-matlabR2020b-v3.4.5')
% addpath('C:\Users\mehre\OneDrive\Desktop\CasADi\casadi-windows-matlabR2016a-v3.4.5')
import casadi.*

x = SX.sym('w'); 
obj =exp(0.2*x).*sin(x); 

g = [];
P = []; 

OPT_variables = x; 
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P); 

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

args = struct;
args.lbx = 0;  % lower bound of the states x and y
args.ubx = 4*pi;   % upper bound of the states x and y 
args.lbg = -inf;  % lower bound of the states x and y
args.ubg = inf;   % upper bound of the states x and y 

args.p = []; 
args.x0 = 10; 

sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p); 
x_sol = full(sol.x)
min_value = full(sol.f)


