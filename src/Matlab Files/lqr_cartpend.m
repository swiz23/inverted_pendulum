clear all, close all, clc

L = .84;
g = 9.81;

A = [0 1 0 0;
    0 0 0 0;
    0 0 0 1;
    0 0 g/L 0];

B = [0; 1; 0; -1/L];
eig(A)

Q = [.3 0 0 0;
    0 .001 0 0;
    0 0 100 0;
    0 0 0 .1];
    
R = 1;
K = lqr(A,B,Q,R)


