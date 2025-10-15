function [FVAL] = B2_6DOF_model_implicit(XDOT,X,U)

FVAL = B2_6DOF_model(X,U) - XDOT;
