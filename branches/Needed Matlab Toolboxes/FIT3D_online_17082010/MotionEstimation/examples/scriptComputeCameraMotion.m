% Author: Isaac Esteban
% IAS, University of Amsterdam
% TNO Defense, Security and Safety
% isaac@fit3d.info
% isaac.esteban@tno.nl
% Copyright TNO - 2010

%% SAMPLE SCRIPT TO COMPUTE THE MOTION OF A CAMERA
% This will show the steps to compute the motion of a camera given some
% images (with no radial distortion)

%% LOAD CAMERA CALIBRATION
load(strcat(install_path,'/MotionEstimation/examples/data/FloriandeSet1/KCanon10mm.mat'));

%% GET SIFT FEATURES
% We use VL_FEAT though regular sift can be employed. Make sure to delete
% all _BW.jpg files from the data folder!!!
fprintf('\n\n\nEXTRACTING SIFT FEATURES\n\n\n');
[F,Files] = getSIFT(strcat(install_path,'/MotionEstimation/examples/data/FloriandeSet1/img/'),8,1,'VL_FEAT');

%% COMPUTE MOTION
% Given the SIFT features, we match them accross frames and compute the
% motion with the specifiec algorithm. See help computeCameraMotion for an
% extended explanation.
fprintf('\n\n\nCOMPUTING MOTION\n\n\n');
[Fplus, PcamPlus, PcamX, FplusExtra,KBA] = computeCameraMotion('VL_FEAT',F, 0.4, 1, 2500, Kcanon10GOOD, 8, true, '8pts', 0.001, 500);

fprintf('NOTE: please make sure that the folder MotionEstimation/examples/data/FloriandeSet1/img/ does ONLY contain the desired images.-\n');

fprintf('------------------------------------------\n');
fprintf('INPUT:\n\n');
fprintf('- Folder with images\n');
fprintf('- Maximum number of frames\n');
fprintf('- Frames to skip each step\n');
fprintf('- Sift method (VL_FEAT is faster)\n');
fprintf('------------------------------------------\n');

fprintf('------------------------------------------\n');
fprintf('OUTPUT:\n\n');
fprintf('- Fplus & FplusExtra (see Doc for details on data structures).\n');
fprintf('- PcamX: n 3x4 camera matrices [R|t]). See Doc for details.\n');
fprintf('- KBA optimized camera calibration for every frame.\n');
fprintf('------------------------------------------\n');

fprintf('\n\n\nThe motion has been computed (and stored in PcamX)!\n See help computeCameraMotion for details on the output.\n\n\n');


