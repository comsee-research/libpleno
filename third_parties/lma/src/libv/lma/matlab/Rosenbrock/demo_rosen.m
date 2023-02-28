% Several minimization of the Rosenbrock function from amost the same
% initialization point

clear all; close all;

% Press any button to go ahead (pause)
%gradient('auto', 0, -1);
%newton('auto', eps, -1);
%simplex('auto', [0,1,1], [-1,0,1]);
levenberg('auto', 0, -1);