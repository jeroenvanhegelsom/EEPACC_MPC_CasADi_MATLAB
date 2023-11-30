# EEPACC_MPC_CasADi_MATLAB
MATLAB simulation of a CasADi implementation of the MPC-based PACCs developed in the Master thesis "Real-Time MPC Strategy for Predictive Adaptive Cruise Control"

Battery electric vehicles (BEVs) are expected to play an important role in the transition
to sustainable energy in the automotive field, and advanced driver assistance systems
can play a big role in optimizing their performance. The key performance indicator
regarding sustainability is a high energy efficiency, which simultaneously increases
the driving range for BEVs.

In this research, energy-efficient longitudinal motion control of a BEV is considered.
More specifically, a proof of concept of an energy efficient predictive adaptive cruise
control (EEPACC) is designed. Currently, most PACCs only consider vehicle following,
but in this report the handling of route aspects and traffic rules such as speed limits,
curves, stops and traffic lights are also incorporated. The EEPACC uses model predictive
control (MPC) strategies to optimize control actions in real-time. Optimization
problems in the form of quadratic programs (QPs) are formulated and implemented
in an MPC framework. Several solvers and problem formulations are considered and
compared. To assess the optimality of the QP-based MPCs, a nonlinear problem (NLP)
is solved to acquire a local optimal solution of the full trajectory over a given route.
First, requirements are formulated that concern safety, traffic compatibility, comfort,
energy-efficiency, driver acceptance and hardware requirements. The requirements
are translated into use cases and key performance indicators (KPIs) that are used to
test the MPCs and directly relate the results to the requirements.

A generic BEV model is discussed that includes the longitudinal dynamics and a powertrain
model. The BEV model is based on a BMW i3 94Ah (2017), and to obtain a
realistic power consumption model, readily available dynamometer measurements
are analyzed and power consumption maps are fitted on the data that are used to
penalize power consumption in some versions of the EEPACC.

An NLP version of the EEPACC is formulated first in MATLAB using the CasADi toolbox,
and this nonlinear problem is translated into a QP that is used to design a force-based
MPC (FBMPC) for an EEPACC. This MPC considers the motor and mechanical braking
forces separately to also include the effects of regenerative braking. A simplified version
of the force-based MPC is the acceleration-based MPC (ABMPC) for which the
dynamics, constraints and objective function are only based on the acceleration of
the vehicle. The acceleration and jerk are penalized to try to optimize for energyefficiency.
The results show that the FBMPC is more energy-efficient in most use cases, which
is related to the braking and acceleration profiles, which are more efficient than the
ones observed in the ABMPC. Both EEPACC versions successfully follow traffic rules
in separate use cases and in a larger scenario and perform as desired in most vehicle
following situations. Real-time capability is tested for both MPCs, and using qpOASES
with the dense QP formulation is found to result in solving times that come close
to real-time capable values with the ABMPC being the quickest. Both MPCs can still
be tuned to improve driving performance both in terms of travel time and energy
consumption, and to reduce complexity of the QPs, which should result in a decrease
of the time required to solve the optimization problems of the MPCs.
