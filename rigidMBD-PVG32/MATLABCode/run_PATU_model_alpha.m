clear all
close all

%% Folders and file paths
addpath('./data/');
addpath('./multibodyLib/');
addpath('../figures/');
addpath('./hydraulicsLib/');

loadCase    = "95KG";                 % folder: "95KG", "180KG", "278KG"
filePath    = "data";
expDataPath = filePath +"/" + "Patu_loadCase_" + loadCase+"_short" + ".mat";
expData     = importdata(expDataPath);

%% Simulation time

Ts          = 1e-3;
tEnd        = 50  ;
[param]     = Get_parameters_Patu("Identified");

%% Sweep settings
bulkType    = param.BulkModulus(1);    % air-content / bulk modulus model to use
valveTypes  = param.valve_models;      % or e.g. param.valve_models(1) for one valve

% Each value is one run: param.cyl(1).alphaB = alphaB_list(k)
alphaB_list = [1/100; 5/100; 15/100; 30/100; 38/100; 50/100];
 
% alphaB_list = [38/100];

simData     = containers.Map();
modelNames  = strings(0);
param.bulkType = bulkType;

for k = 1:numel(alphaB_list)

    param.cyl(1).alphaB = alphaB_list(k);
    caseTag             = sprintf("aB%02d", round(100*alphaB_list(k)));   % e.g. aB05 = 5 %

    for i = 1:numel(valveTypes)

        param.valveType = valveTypes(i);
        modelName       = param.valveType+'_'+param.bulkType+'_'+caseTag;

        simData(modelName) = true_hydraulics_PATU(expData, tEnd, param, loadCase );
        modelNames(end+1)  = modelName;

    end
end

%% Plotting (func_plot writes one CSV per signal and run into figDir)
figDir          = fullfile("..", "figures", "alphaB_sweep", loadCase + "_" + bulkType);   % one folder per load and air content
if ~exist(figDir, 'dir'); mkdir(figDir); end
mode            = 'identified';
nSteps          = numel(simData(modelName).Time);
nStart          = 10;
tspan           = 0:Ts:Ts*(nSteps-nStart);
[color,plots]   = Get_colors_plots();

colors          = [color.blue;color.green;color.purple; .....
    color.deeporange;color.brass;color.Golden ];

func_plot(simData, modelNames, expData, plots, ...
    colors, tspan, nStart, nSteps, figDir,mode);
