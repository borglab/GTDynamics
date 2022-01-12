# Chain Class

## Motivation
GTDynamics is very expressive in including all link kinematics and dynamics variables into a large factor graph. However, sometimes we know that we are specifically *not* interested in some links or, more generally, a chain of links. 

The motivating example is the legs of a quadruped: we are really only interested in what torque they can deliver to the "actuated potato" that constitutes the quadruped body.

## Notation

Below I use tensor notation with superscripts and subscripts, and make sure matrix multiplication respects contraction rules. Consequently, I use $\theta^j$ for joint angles, with $\theta^1$ the first joint angle, etc. Also, I use $[v]$ rather than $\hat{v}$ to lift a vector $v$ to the Lie algebra $\mathfrak{se}(3)$.

## Kinematics

We know that the forward kinematics of a chain can be given by a **product of exponential maps** or POE, e.g., in the 3DOF case we have
$$T^b_e(\theta) = M^b_e \exp[A^e_1\theta^1]\exp[A^e_2\theta^2]\exp [A^e_3\theta^3] \doteq M^b_e \exp[A^e_j\theta^j]$$
where the $6\times3$ matrix $A^e_j$ collects the joint screw axes expressed in the end-effector frame. Note I defined $\exp[A^e_j\theta^j]$ as shorthand for the POE of all axes in the matrix $A^e_j$.

In fact, $A^e_j$ is just the **manipulator Jacobian** $J^e_j(\theta^j)$ at rest, and more generally we have
$$T^b_e(\theta^j+\delta \theta^j) = T^b_e(\theta^j) \exp[J^e_j(\theta^j)\delta \theta^j]$$
with $A^e_j=J^e_j(0^j)$.

## Monoid Math

We can define the **chain** $S^b_{e,j} \doteq (M^b_e, A^e_j)\in\mathcal{S_j}$ as specifying a chain kinematic chain with end-effector pose $M^b_e$ at rest, expressed in the body frame $B$, and screw axes $A^e_j$ for joint angles indexed by $j$, expressed in the end-effector frame $E$. We also define $\mathcal{S_j}$ as the space of all such chains with joint angles index by $j$.

I also define the *compose* operation $*$ between chains $S^b_{e,j}$ and $S^e_{f,k}$ as
$$S^b_{e,j} * S^e_{f,k} = (M^b_e, A^e_j) * (M^e_f, A^f_k) =( M^b_e M^e_f, Ad_{M^f_e} A^e_j | A^f_k) = S^b_{f,j+k}.$$
This operation can be used to compose chain kinematic chains effortlessly, as long as the second arm is expressed in the end-effector frame $E$ of the first. It makes sure that the screw axes $A^f_{j+k}$ of the composed arm are all expressed in the end-effector frame $F$ of the composed arm.

The operation $*$ is associative, and has an identity element $I^s_{s,\_} \doteq (I_{SE(3)}, 0^s_\_)$, with $S$ an arbitrary frame and $0^s_\_$ a $6\times0$ matrix, and hence forms a $monoid$, i.e., a group without an inverse.

## Actuated Chains

Given a chain $S^b_{e,j}=(M^b_e, A^e_j)$ we can define a forward kinematics function $S^b_{e,j}(\theta^j): \mathcal{S_j} \times R^n \rightarrow \mathcal{S_j}$ that produces an actuated end-effector pose along with its associated, configuration-specific Jacobian:
$$S^b_{e,j}(\theta^j) \doteq (M^b_e \exp[A^e_j\theta^j], J^e_j(\theta^j))$$

The compose operation *also* works for actuated chains, and hence can be used to compute configuration-specific Jacobians:
$$S^b_{f,j+k}(\theta^j, \theta^k) = S^b_{e,j}(\theta^j) * S^e_{f,k}(\theta^k)$$

## Simplified Dynamics

> This is not fully worked/correct out yet because, in contrast to $SE(2)$, in $SE(3)$ a $6\times3$ Jacobian is obviously not invertible and there must be null-spaces involved, or excess wrench has to be resisted by the ground/mechanism, etc...
> 
For a *massless* leg, the Jacobian also describes the relationship between the torques $\tau_j$ applied at the joints, and the wrench $\mathcal{F_e}$ applied at the end-effector:

$$\tau_j = \mathcal{F_e} J^e_j(\theta).$$

Note, above we treat $\tau_j$ and $\mathcal{F_e}$ as *row* vectors, as indicated by the subscripts $j$ and $e$. In particular, $\tau_j$ is a $1\times3$ vector, and $F_e$ is a $1\times6$ vector.

Of course, we are typically more interested in applying a wrench $\mathcal{F_b}$ on the body $B$, by delivering torques $\tau_j$ ate the joints, so we need to adjoint from the body:
$$\tau_j = \mathcal{F_b} Ad^b_e J^e_j(\theta)$$
with
$$Ad^b_e\doteq Ad_{T^b_e}.$$

Note that if in GTSAM we want all vectors as column vectors, we get the familiar L&P math:

$$\tau_j^T =  J^e_j(\theta)^T [Ad_{T^b_e}]^T \mathcal{F_b}^T$$


