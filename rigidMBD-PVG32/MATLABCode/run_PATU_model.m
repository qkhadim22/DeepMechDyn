clear all
close all

%% Folders and file paths 
addpath('./data/');
addpath('./multibodyLib/');
addpath('../figures/');
addpath('./hydraulicsLib/');

loadCase = "180KG";                 % folder: "95KG", "180KG", "278KG"
filePath    = "data";
expDataPath = filePath +"/" + "Patu_loadCase_" + loadCase+"_short" + ".mat";
expData     = importdata(expDataPath);

%% Simulation time

Ts          = 1e-3;
tEnd        = 40  ;
[param]     = Get_parameters_Patu("Standard");
simData     = containers.Map();
modelNames  = strings(0);

param.cyl(1).alphaB = [1/100; 5/100; 15/100;30/100; 38/100;50/100];


for i = 1:numel(param.valve_models)

    param.valveType = param.valve_models(i);

    for j = 1:numel(param.BulkModulus)

        param.bulkType = param.BulkModulus(j);
        modelName      = param.valveType+'_'+param.bulkType;

        simData(modelName)     = true_hydraulics_PATU(expData, tEnd, param, loadCase );
        modelNames(end+1)      = modelName;

        %sim                = simData(k);
        %fullFileName     = fullfile(filePath, sim.Name + ".mat");
        %save(fullFileName, "sim");

    end
end

%% Plotting 
figDir          = '../figures/';
mode            = 'identified';
nSteps          = numel(simData(modelName).Time);
nStart          = 10;
tspan           = 0:Ts:Ts*(nSteps-nStart);
[color,plots]   = Get_colors_plots();

colors          = [color.blue;color.green;color.purple; .....
    color.deeporange;color.brass;color.Golden ];

func_plot(simData, modelNames, expData, plots, ...
    colors, tspan, nStart, nSteps, figDir,mode);


