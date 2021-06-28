% Demo: Fit points with Fourier Serise.
% Author: Jiaheng Zhao

clc;
clear all;
close all;

%load data: ori_points;
load(fullfile(pwd,filesep,'Data',filesep,'demo_fsFitting.mat'));

f1 = figure;
axis([-5 5 -5 5])
axis equal
% feature's shape -- groundtruth
pt1 = plot(ori_points(1,:),ori_points(2,:),'k'); 
hold on;

% sample some points for fitting.
smpID = 100 : 5 : 200;   % What will it look like if changing smpID? Try.
pt2 = plot(ori_points(1,smpID),ori_points(2,smpID),'bo','MarkerSize',12); 

% Number of the coefficient.
N = 5;  % It should be less than the size of smpID.
isComplement = true; % set to true if the observed points are not sufficient. For example, smpID = 5:20:200

%% -------------------- This is the function you need to complete --------------------------%


[V, center, newpt] = fitWithFS( N, ori_points(1,smpID), ori_points(2,smpID), [], isComplement);


%% -----------------------------------------------------------------------------------------------%
% Plot
pt3 =  plot(newpt(1,:),newpt(2,:),'r*','MarkerSize',12); 
fs1 = plotFS(center,V,'r');
pct = plot(center(1),center(2),'rp','MarkerSize',12);