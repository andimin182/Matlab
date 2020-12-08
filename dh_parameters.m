%%% DENAVIT HARTENBERG Parameters %%%
clear all
close all
clc
% Window title
prompt = {'Insert how many robot links'};
dlg_title = 'Input Links';
num_lines = 1;
def_input = {'1'}; % default value
% Window properties
opts.Resize = 'on';
opts.WindowStyle = 'normal';
answer = inputdlg(prompt,dlg_title,num_lines,def_input, opts);
num_link = str2num(answer{:});
F = sym('A', [num_link 4]);
B=eye(4);
C = sym('C', [4 4]);
clc
for i=1:num_link
    prompt = {'Enter a:','Enter \alpha:','Enter d:','Enter \theta:'};
    dlg_title = sprintf('Link%d',i);
    num_lines = 1;
    opts.Resize = 'on';
    opts.Interpreter = 'tex';
    def1 = {sprintf('a%d',i),sprintf('alpha%d',i),sprintf('d%d',i),sprintf('q%d',i)};
    answer1 = inputdlg(prompt,dlg_title,num_lines,def1, opts);
    F(i,1)=answer1(1,1);
    F(i,2)=answer1(2,1);
    F(i,3)=answer1(3,1);
    F(i,4)=answer1(4,1);
    C=simplify([cos(F(i,4)) -sin(F(i,4))*cos(F(i,2)) sin(F(i,4))*sin(F(i,2)) F(i,1)*cos(F(i,4));
             sin(F(i,4)) cos(F(i,4))*cos(F(i,2)) -cos(F(i,4))*sin(F(i,2)) F(i,1)*sin(F(i,4));
             0 sin(F(i,2)) cos(F(i,2)) F(i,3);
             0 0 0 1]);
    eval(sprintf('A%d = C;',i));
    B=B*C;
    eval(sprintf('A%d',i))
 
end
sprintf('T from Arm 0 to Arm %d is:',i)
pretty(simplify(B))
'R is Rotation Matrix'  , R=B(1:3,1:3);
pretty(R)
p=B(1:3,4);
'Direct Kinematics p = f(q)' , pretty(p)

% ---- Additional inputs: transformation from world to link1 and from last
% link to EE
   
T0W = [0 0 1 0;
       1 0 0 0;
       0 1 0 0;
       0 0 0 1];
   
TE2 = [0 0 1 0;
       0 -1 0 0;
       1 0 0 0;
       0 0 0 1];
   
 Btot = T0W*B*TE2;
 
 'R is total rotation matrix' , Rtot =Btot(1:3,1:3);
 pretty(Rtot)
 
 ptot = Btot(1:3,4);
 'Direct Kinematics wrt world: p = f(q)' , pretty(ptot)
 
 %%save('A_matrices', 'A1', 'A2', 'num_link')