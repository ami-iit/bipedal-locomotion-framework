# 🧮 SO3 Minimum jerk trajectory {#so3-minjerk}

## Introduction
Planning a minimum jerk trajectory in SO(3) can be a complex task. In this document, we aim to demonstrate the capabilities of the **bipedal-locomotion-framework** for generating minimum jerk trajectories in SO(3). The document is structured as follows: we first provide an overview of the mathematical foundations behind the planner, followed by a simple example illustrating its usage. Finally, we recommend some interesting readings for readers seeking more in-depth explanations and rigorous proofs.

## 📐 Math
Given a fixed initial rotation \f$(t_0, R_0)\f$ and a final rotation \f$(t_f, R_f)\f$ and the associated left trivialized angular velocities, \f$(t_0, \omega_0)\f$ and \f$(t_f, \omega_f)\f$,  we want to compute a trajectory \f$R : \mathbb{R}_+ \rightarrow SO(3)\f$ such that \f$R(t_0) = R_0\f$, \f$R(t_f) = R_f\f$,  \f$\omega(t_0) = \omega_0\f$, \f$\omega(t_f) = \omega_f\f$,  \f$\dot{\omega}(t_0) = \dot{\omega}_0\f$, \f$\dot{\omega}(t_f) = \dot{\omega}_f\f$ such that left trivialized angular jerk is minimized

\f[
\mathfrak{G} = \int_{t_0}^{t_f} \left({}^\mathcal{F} \ddot{\omega} _ {\mathcal{I}, F}^\top {}^\mathcal{F} \ddot{\omega} _ {\mathcal{I}, F}  \right)\text{d} t.
\f]

Following the work of [Zefran et al. "On the Generation of Smooth Three-Dimensional Rigid Body Motions"](https://doi.org/10.1109/70.704225) it is possible to prove that a trajectory \f$R(t)\f$ that satisfies the following equation is a minimum jerk trajectory in SO(3).

\f[
\begin{array}{ll}
\omega ^{(5)} & + 2 \omega \times \omega ^{(4)} \\
              & + \frac{4}{5} \omega \times (\omega  \times \omega ^{(3)}) + \frac{5}{2} \dot{\omega} \times \omega^{(3)} \\
              & + \frac{1}{4} \omega \times (\omega \times (\omega \times \ddot{\omega})) \\
              & + \frac{3}{2} \omega \times (\dot{\omega} \times \ddot{\omega}) \\
              & - (\omega \times \ddot{\omega}) \times \dot{\omega} \\
              & - \frac{1}{4} (\omega \times \dot{\omega}) \times \ddot{\omega} \\
              & - \frac{3}{8} \omega \times ((\omega \times \dot{\omega}) \times \dot{\omega}) \\
              & - \frac{1}{8} (\omega \times (\omega \times \dot{\omega})) \times \dot{\omega}  = 0.
\end{array}
\f]

From now on we call this condition: **Minimum jerk trajectory condition**.

It is worth noting that the above does not admit an analytic solution for arbitrary boundary conditions.
However, in the case of zero boundary velocity and acceleration or specific structure of the boundary condition, it is possible to show that
\f[
R(t) =  R_{0} \exp{\left(s(t-t_0) \log\left(R_0^\top R_f \right) \right)} \quad \quad s(\tau) = a_5 \tau^5 + a_4 \tau^4 + a_3 \tau^3 + a_2 \tau^2 + a_1 \tau + a_0
\f]
satisfies condition the minimum jerk necessary condition. To prove the latest statement, we assume that \f$t_0 = 0\f$ and \f$t_1 = 1\f$, and then we compute the left trivialized angular velocity. The assumption can be easily removed with a simple change of variables.

The angular velocity is given by

\f[
\begin{array}{ll}
\omega^\wedge &= R^\top \dot{R} \\
&= \dot{s} \exp{\left(-s\log\left( R _ 0^\top R _ f  \right) \right)}R_{0} ^\top R _ {0} \log\left( R _ 0^\top R _ f  \right) \exp{\left(s\log\left( R _ 0^\top R _ f \right) \right)} \\
&= \dot{s}\exp{\left(-s\log\left(R _ 0^\top R _ f  \right) \right)} \log\left( R _ 0^\top R _ f \right) \exp{\left(s\log\left( R _ 0^\top R _ f  \right) \right)} \\
&= \dot{s}\exp{\left(-s\log\left(R _ 0^\top R _ f  \right) \right)} \exp{\left(s\log\left( R _ 0^\top R _ f  \right) \right)}  \log\left( R _ 0^\top R _ f \right) \\
&= \dot{s} \log\left( R _ 0^\top R _ f \right).
\end{array}
\f]

In other words, the angular velocity will be always proportional to \f$\log\left( R_0^\top R_f \right)\f$.

This means that we can ask for an initial and final angular velocity that is linearly dependent on \f$\log\left(R_0^\top R_f \right)\f$.
Let us introduce \f$\omega^\wedge(0)\f$ and \f$\omega^\wedge(1)\f$ as

\f[
\omega^\wedge(0) = \lambda ^\omega _ 0 \log\left( R_0^\top R_f \right) \quad \omega^\wedge(1) = \lambda ^\omega _ 1 \log\left( R_0^\top R_f \right)
\f]

similarly for the angular acceleration

\f[
\dot{\omega}^\wedge(0) = \lambda ^\alpha _ 0 \log\left( R_0^\top R_f \right) \quad \dot{\omega}^\wedge(1) = \lambda ^\alpha _ 1 \log\left( R_0^\top R_f \right)
\f]

So combining the initial and the final boundary conditions we can write the following linear system.
\f[
\begin{array}{ll}
s(0) &= 0 \\
s(1) &= 1 \\
\dot{s}(0) &= \lambda ^\omega _ 0 \\
\dot{s}(1) &= \lambda ^\omega _ 1 \\
\ddot{s}(0) &= \lambda ^\alpha _ 0 \\
\ddot{s}(1) &= \lambda ^\alpha _ 1
\end{array}
\f]

Solving the above system  we obtain

\f[
\begin{array}{ll}
a_0 &= 0 \\
a_1 &= \lambda ^\omega _ 0  \\
a_2 &= \lambda ^\alpha _ 0/2 \\
a_3 &= \lambda ^\alpha _ 1/2 - (3 \lambda ^\alpha _ 0)/2 - 6 \lambda ^\omega _ 0 - 4 \lambda ^\omega _ 1 + 10    \\
a_4 &= (3 \lambda ^\alpha _ 0)/2 - \lambda ^\alpha _ 1 + 8 \lambda ^\omega _ 0 + 7 \lambda ^\omega _ 1 - 15      \\
a_5 &= \lambda ^\alpha _ 1/2 - \lambda ^\alpha _ 0/2 - 3 \lambda ^\omega _ 0 - 3 \lambda ^\omega _ 1 + 6
\end{array}
\f]

It is now easy to show that \f$\omega\f$ satisfies the minimum jerk condition indeed

\f[
\omega ^{(5)} = 0
\f]

and all the cross products vanish since the angular velocity and its derivatives are linearly dependent.

## 💻 How to use the planner

The SO3 minimum jerk planner is provided in **bipedal-locomotion-framework** through a template class that allows the user to use the left or the right trivialized angular velocity

```cpp
using namespace std::chrono_literals;

manif::SO3d initialTransform = manif::SO3d::Random();
manif::SO3d finalTransform = manif::SO3d::Random();

constexpr std::chrono::nanoseconds T = 1s;

BipedalLocomotion::Planner::SO3PlannerInertial planner;
manif::SO3d::Tangent initialVelocity = (finalTransform * initialTransform.inverse()).log();

initialVelocity.coeffs() = initialVelocity.coeffs() * 2;
planner.setInitialConditions(initialVelocity, manif::SO3d::Tangent::Zero());
planner.setFinalConditions(manif::SO3d::Tangent::Zero(), manif::SO3d::Tangent::Zero());
planner.setRotations(initialTransform, finalTransform, T);

manif::SO3d rotation, predictedRotation;
manif::SO3d::Tangent velocity, predictedVelocity;
manif::SO3d::Tangent acceleration;
planner.evaluatePoint(0.42s, rotation, velocity, acceleration);
```

## 📖 Interesting readings
The interested reader
can refer to the extensive literature, among which it is worth mentioning:

- Žefran, M., Kumar, V., and Croke, C. (1998). _On the generation of smooth threedimensional rigid body motions_. IEEE Transactions on Robotics and Automation,
14(4):576–589.
- Dubrovin, B. A., Fomenko, A. T., and Novikov, S. P. (1984). _Modern Geometry —
Methods and Applications, volume 93_. Springer New York, New York, NY.
- Needham, T. (2021). _Visual Differential Geometry and Forms_. Princeton University
Press.
- Pressley, A. (2010). _Elementary Differential Geometry_. Springer London, London.
- Giulio Romualdi (2022) _Online Control of Humanoid Robot Locomotion_ Ph.D. Thesis.
