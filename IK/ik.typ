#align(center)[
    = Inverse Kinematics

    #v(2cm)

    #figure(
    image("ik_diagram.png", width: 70%),
    caption: none
    )
]

#pagebreak()

Solving for $theta_{2}$ and $theta_{3}$ when $(x, z)$ is known

#v(5mm)

1. Solve for $L_12$
$L_12 = sqrt(x^2 + z^2)$ #h(1fr) Pythagorean theorem

#v(5mm)

2. Solve for $theta_12$
$theta_12 = cos^(-1)(frac(L_1^2 + L_2^2 - L_12^2, 2 times L_1 times L_2))$ #h(1fr) Law of cosines

#v(5mm)

3. Solve for $theta_3$
$theta_3 = 180 - theta_12$ #h(1fr) Supplementary angles

4. Solve for $alpha_1$
$alpha_1 = tan^(-1)(frac(x, z))$

5. Solve for $gamma_1$
$gamma_1 = sin^(-1)(frac(sin theta_12 times L_2, L_12))$ #h(1fr) Law of sines

6. Solve for $theta_2$
$theta_2 = 90 - (alpha_1 + gamma_1)$ #h(1fr) Complementary angles