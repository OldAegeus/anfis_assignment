% ANFIS networks for solving 3RR planar manipulator
%% Choose parameters
clc, cla

%Split of data (validation set is 1-trn_data_per)
trn_data_per = 0.5;


%Link lengths
l_1 = 10;
l_2 = 7;
l_3 = 7;


%Theta ranges
th_1 = 0:0.05:pi/2;
th_2 = 0:0.1:pi;
th_3 = -pi/2:0.1:pi/2;

%ANFIS Epochs

th_1_epoch = 200;
th_2_epoch = 200;
th_3_epoch = 200;

%% Generate data

%Randomise theta array order
th_1_numl = numel(th_1);
th_1 = th_1(randperm(th_1_numl, end));
th_2_numl = numel(th_2);
th_2 = th_2(randperm(th_2_numl, end));
th_3_numl = numel(th_3);
th_3 = th_3(randperm(th_3_numl, end));


%Break data into training and validation sets
th_1_spl = floor(trn_data_per*th_1_numl);
th_2_spl = floor(trn_data_per*th_2_numl);
th_3_spl = floor(trn_data_per*th_3_numl);

th_1_trn = th_1(1:th_1_spl);
th_2_trn = th_2(1:th_2_spl);
th_3_trn = th_3(1:th_3_spl);

th_1_val = th_1(th_1_spl+1:end);
th_2_val = th_2(th_2_spl+1:end);
th_3_val = th_3(th_3_spl+1:end);


%Convert to grids
[th_1_trn, th_2_trn, th_3_trn] = ndgrid(th_1_trn, th_2_trn, th_3_trn);
[th_1_val, th_2_val, th_3_val] = ndgrid(th_1_val, th_2_val, th_3_val);


%Use forward kinematics to determine end-effector positions
x_trn = l_1*cos(th_1_trn) + l_2*cos(th_1_trn+th_2_trn) + l_3*cos(th_1_trn+th_2_trn+th_3_trn);
y_trn = l_1*sin(th_1_trn) + l_2*sin(th_1_trn+th_2_trn) + l_3*sin(th_1_trn+th_2_trn+th_3_trn);
phi_trn = th_1_trn + th_2_trn + th_3_trn;

x_val = l_1*cos(th_1_val) + l_2*cos(th_1_val+th_2_val) + l_3*cos(th_1_val+th_2_val+th_3_val);
y_val = l_1*sin(th_1_val) + l_2*sin(th_1_val+th_2_val) + l_3*sin(th_1_val+th_2_val+th_3_val);
phi_val = th_1_val + th_2_val + th_3_val;


%Form complete datasets
th_1_data_trn = [x_trn, y_trn, phi_trn, th_1_trn];
th_2_data_trn = [x_trn, y_trn, phi_trn, th_2_trn];
th_3_data_trn = [x_trn, y_trn, phi_trn, th_3_trn];

th_1_data_val = [x_val, y_val, phi_val, th_1_val];
th_2_data_val = [x_val, y_val, phi_val, th_2_val];
th_3_data_val = [x_val, y_val, phi_val, th_3_val];

%% Plot the training set vs validation sets
figure(1);
plot(x_trn(:), y_trn(:), 'r.', 'DisplayName','Training Set'); 
hold on;
plot(x_val(:), y_val(:), 'b.', 'DisplayName','Validation Set'); 
hold on;
axis equal;
xlabel('X')
ylabel('Y')
title(['Training (' num2str(trn_data_per*100) '%) vs. Validation Points. (' num2str((1-trn_data_per)*100) '%)' ]);
legend;
saveas(figure(1),['trnvsval' num2str(trn_data_per*100) '.jpg'])

%% Generate the initial fuzzy inference system

th_1_fismat = genfis2(th_1_data_trn(:,1:3), th_1_data_trn(:,4),0.2);
th_2_fismat = genfis2(th_1_data_trn(:,1:3), th_1_data_trn(:,4),0.2);
th_3_fismat = genfis2(th_1_data_trn(:,1:3), th_1_data_trn(:,4),0.2);

%%

fprintf('-->%s\n','Training theta_1 ANFIS network.')
opt = anfisOptions('InitialFIS',th_1_fismat,'EpochNumber',th_1_epoch);
opt.DisplayANFISInformation = 0;
opt.DisplayErrorValues = 0;
opt.DisplayStepSize = 0;
opt.DisplayFinalResults = 0;
opt.ValidationData = th_1_data_val;
[th_1_anfis,th_1_trnErr,th_1_ss,th_1_valAnfis,th_1_valErr] = anfis(th_1_data_trn,opt);

fprintf('-->%s\n','Training theta_2 ANFIS network.')
opt = anfisOptions('InitialFIS',th_2_fismat,'EpochNumber',th_2_epoch);
opt.DisplayANFISInformation = 0;
opt.DisplayErrorValues = 0;
opt.DisplayStepSize = 0;
opt.DisplayFinalResults = 0;
opt.ValidationData = th_2_data_val;
[th_2_anfis,th_2_trnErr,th_2_ss,th_2_valAnfis,th_2_valErr] = anfis(th_2_data_trn,opt);

fprintf('-->%s\n','Training theta_3 ANFIS network.')
opt = anfisOptions('InitialFIS',th_3_fismat,'EpochNumber',th_3_epoch);
opt.DisplayANFISInformation = 0;
opt.DisplayErrorValues = 0;
opt.DisplayStepSize = 0;
opt.DisplayFinalResults = 0;
opt.ValidationData = th_3_data_val;
[th_3_anfis,th_3_trnErr,th_3_ss,th_3_valAnfis,th_3_valErr] = anfis(th_3_data_trn,opt);
