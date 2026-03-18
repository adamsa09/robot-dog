#align(center)[
    = Quadruped Robot Dog
    == Adam Sarhan & Lucas Dauth
    #v(2cm)


]

#pagebreak()

== Inverse Kinematics
    #figure(
    image("assets/ik_diagram.png", width: 70%),
    caption: none
    )
Solving for $theta_2$ and $theta_3$ when $(x, z)$ is known

#v(3mm)

1. $L_23$
$L_23 = sqrt(x^2 + z^2)$ #h(1fr) Pythagorean theorem

#v(3mm)

2. $theta_23$
$theta_23 = cos^(-1)(frac(L_2^2 + L_3^2 - L_23^2, 2 times L_1 times L_3))$ #h(1fr) Law of cosines

#v(3mm)

3. $theta_3$
$theta_3 = 180 - theta_23$ #h(1fr) Supplementary angles

4. $alpha_1$
$alpha_1 = tan^(-1)(frac(x, z))$

5. $gamma_1$
$gamma_1 = sin^(-1)(frac(sin theta_23 times L_3, L_23))$ #h(1fr) Law of sines

6. $theta_2$
$theta_2 = 90 - (alpha_1 + gamma_1)$ #h(1fr) Complementary angles\
\ 
$theta_2$ is written to the hip joint servo\ 
$theta_3$ is written to the knee joint servo

#pagebreak()
== Gait
Box step. Linear interpolation for motion smoothing.\ \  
$P_i = \{(50,200), (50,155), (-50,155), (-50,200), (50,200)\}$

#table(
  columns: (auto, auto, auto, auto),
  align: (center, center, center, left),
  table.header(
    [$P$], [*Phase* $phi$], [*Coordinates*], [*Description*],
  ),
  [0], [0],    [(50, 200)],   [Foot at resting position],
  [1], [0.25], [(50, 155)],   [Highest lift],
  [2], [0.50], [(-50, 155)],  [Maximum forward reach],
  [3], [0.75], [(-50, 200)],  [Back to floor],
  [4], [1],    [(50, 200)],   [Return],
)

=== Linear interpolation
Given the active leg phase $phi_"leg"$, bounding keypoints $P_i$ and $P_j$ are
selected such that $phi_i <= phi_"leg" < phi_j$. The normalized interpolation
parameter is:

$alpha = frac(phi_"leg" - phi_i, phi_j - phi_i), quad alpha in [0, 1]$

Foot coordinates are then computed as:

$x(phi) = x_i + alpha (x_j - x_i)$

$z(phi) = z_i + alpha (z_j - z_i)$

The cycle closes by wrapping the segment from $P_3$ back to $P_0$. When
$phi_"leg" >= phi_3$ or $phi_"leg" < phi_0$, the phase end is extended:

$phi_j^* = phi_0 + 1.0$

and the effective phase is shifted if $phi_"leg" < phi_0$:

$phi_"eff" = phi_"leg" + 1.0$

The global phase advances each loop iteration by:

$Delta phi = frac(Delta t_"ms", T_"period") dot s$

where $T_"period" = 1000$ ms and $s in [0, 3]$ is mapped from the left analog
stick. Phase is then wrapped back into $[0, 1)$:

$phi_(k+1) = (phi_k + Delta phi) mod 1$
