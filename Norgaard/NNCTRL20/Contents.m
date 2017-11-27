%---------------------------
% FUNCTIONS AND SCRIPT FILES
%         IN THE
%     NNCTRL TOOLKIT
%--------------------------
%
%The NNCTRL toolkit consists of a "root" directory and
%three subdirectories. The contents of each directory is
%listed below.
% 
%Overview of files in directory NNCTRL (root directoy):
%------------------------------------------------------
%Contents : This file.
%RELEASE  : Release Notes.
%README   : Readme file.
%manual.ps: Manual for the toolkit.
%Fblin    : Readme file for control by feedback linearization.
%Feedforw : Readme file for feedforward control.
%Imc      : Readme file for internal model control.
%Instlin  : Readme file for control by intsantaneous linearization.
%Inverse  : Readme file for inverse model training and direct inverse control. 
%Npc      : Readme file for nonlinear predictive control.
%Optim    : Readme file for optimal control.
%
%
%Overview of files in directory CTRLTOOL:
%----------------------------------------
%apccon   : Approximate generalized predictive control.
%dio      : Prepare polynomials for 'diophant' in pole placement design.
%diophant : Solves general Diophantine equation.
%experim  : Performs an "experiment" to collect a set of training data.
%fblcon   : Control by feedback linearization.
%ffcon    : Simulation of PID-control optimized with neural feedforward.
%general  : General training of inverse models with Levenberg-Marquardt.
%imccon   : Internal model control simulation program.
%invcon   : Direct inverse control simulation program.
%invsim   : Evaluate an inverse model trained with "general".
%lincon   : Approximate pole placement + minimum variance control.
%npccon1  : Nonlinear predictive control (Quasi-Newton version).
%npccon2  : Nonlinear predictive control (Levenberg-Marquardt version).
%optcon   : Optimal control simulation program.
%opttrain : Trains a network to act as the optimal controller. 
%progress : Displays the completed fraction of a simulation.
%prs      : Generates an vector with a signal that is useful in experiments.
%shift    : Shifts all elements in a vector and inserts a new value.
%siggener : Signal generator.
%special1 : Specialized training of inverse models with back-propagation.
%special2 : Specialized training of inverse models with RPLR Gauss-Newton.
%special3 : Specialized training of inverse models with RPEM Gauss-Newton.
%
%
%Overview of files in directory CTRLDEMO:
%----------------------------------------
%fbltest  : Demonstration of feedback linearization.
%   fblinit - Initialization file used by "fbltest".
%invtest  : Demonstration of direct inverse control.
%   expinit, invinit2, invinit - Initialization files used by "invtest".
%lintest  : Demonstration of instantaneous linearization based control.
%   invinit - Initialization file used by "lintest".
%opttest  : Demonstration of optimal control.
%   opttrinit, optinit - Initialization files used by "opttest".
%predtest : Demonstration of predictive control.
%   npcinit, apcinit - Initialization files used by "predtest".
%ffinit   : Initialization file for feedforward demo. (call "ffcon")
%imcinit  : Initialization file for internal model control (call "imccon")
%spm1     : SIMULINK model of nonlinear process used in the demos.
%springm, smout : MATLAB model of the same process.
%
%
%Overview of files in directory TEMPLATE:
%----------------------------------------
%apcinit  : Template initialization file for "apccon".
%expinit  : Template initialization file for "expcon".
%fblinit  : Template initialization file for "fblcon".
%ffinit   : Template initialization file for "ffcon".
%imcinit  : Template initialization file for "imccon".
%invinit  : Template initialization file for "invcon".
%invinit1 : Template initialization file for "special1".
%invinit2 : Template initialization file for "special2" and "special3".
%lininit  : Template initialization file for "lincon".
%npcinit  : Template initialization file for "npccon1" and "npccon2".
%optinit  : Template initialization file for "optcon".
%optrinit : Template initialization file for "opttrain".
