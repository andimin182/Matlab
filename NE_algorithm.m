%%% --------Implementation of Newton-Euler algorithm ----------------------
fprintf(' Newton-Euler algorithm for computation in real-time')

%% Robot data
% Gravity vector definition
g0 = [0;0;9.81];

% Joint vector and joint velocities vector
q = sym('q', [1 num_link]);
q_dot = sym('q_dot', [1 num_link]);

% Rotation vetor z always equal = [0;0;1]
z = [0;0;1];

%%
% Convert the matrices A in cell. I will directly create A with cell
% structure in dh_parameters.m
transform_cell = cell(1, num_link);
transform_cell{1} = A1;
transform_cell{2} = A2;


% Define a cell that contains the rotation matrices
rotation_cell = cell(1, num_link);

% Define a cell for the position of RF i expressed in i-1
position_cell = cell(1, num_link);

% Extract the rotation matrices from Transforms A:
for i = 1: num_link
    rotation_cell{i} = transform_cell{i}(1:3, 1:3);
    position_cell{i} = transform_cell{i}(1:3, 4);
end
    


%% Consider just REVOLUTE JOINTS sigma = 0. It must be generalized later
% create a cell of omega vectors 3x1, with the first vector initialized to 0
omega = cell(1, num_link +1);
omega{1} = [0;0;0]; 

% create a cell of omega_dot vectors 3x1 initialized to 0
omega_dot = cell(1, num_link+1);
omega_dot{1} = [0;0;0]; 
% create a cell of acc vectors 3x1 initialized to 0. 
% We include as well the gravity acc vector g0=[0,0,9.81] acting on z axis

acc = cell(1, num_link+1);
acc{1} = -g0;


%% FORWARD RECURSION LOOP for computing the accelerations and velocities
for i = 2: (num_link +1)
    omega{i} = transpose(rotation_cell{i-1})*(omega{i-1} + q_dot(i-1)*z); 
    omega_dot{i} = transpose(rotation_cell{i-1}
end









