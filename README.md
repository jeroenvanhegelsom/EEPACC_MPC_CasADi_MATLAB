# EEPACC_MPC_CasADi_MATLAB
MATLAB simulation of a CasADi implementation of the MPC-based PACCs developed in the Master thesis "Real-Time MPC Strategy for Predictive Adaptive Cruise Control"

Battery electric vehicles (BEVs) are expected to play an important role in the transition
to sustainable energy in the automotive field, and advanced driver assistance systems
can play a big role in optimizing their performance. The key performance indicator
regarding sustainability is a high energy efficiency, which simultaneously increases
the driving range for BEVs.

In this research, longitudinal motion control of a BEV is considered, more specifically
predictive adaptive cruise control (PACC). Currently, most PACCs only consider vehicle
following, but in this report the handling of route aspects and traffic rules such as
speed limits, curves, stops and traffic lights are also incorporated.
The PACCs use model predictive control (MPC) strategies with the aim of optimizing
control actions in real-time. Optimization problems in the form of quadratic programs
(QPs) are formulated and implemented in an MPC framework. Several solvers and
problem formulations are considered and compared. To assess the optimality of the
QP-based MPCs, a nonlinear problem (NLP) is solved to acquire a local optimal solu-
tion of the full trajectory.

Requirements are formulated first, to ... safety, traffic compatibility, comfort, energy-
efficiency, driver acceptance and hardware requirements. Key performance indica-
tors (KPIs) are established based on the formulated requirements such that the con-
troller’s results can be assessed and associated with this, several use cases are defined
that are meant to test whether the requirements are met.

A generic BEV model is discussed that includes longitudinal dynamics and powertrain
modeling. A BMW i3 94Ah (2017) model is used for the results. To obtain a realistic
power consumption model, readily available dynamometer measurements are an-
alyzed and power consumption maps is fitted on the data that can be used in the
controllers.
