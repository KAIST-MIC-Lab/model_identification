clear

syms Cf Cr m Vx lf lr Iz


A = [
    -(Cf+Cr)/m/Vx, -(Cf*lf-Cr*lr)/m/Vx/Vx-1
    -(Cf*lf-Cr*lr)/Iz,  -(Cf*lf*lf+Cr*lr*lr)/Iz/Vx
];

B = [
    Cf/m/Vx
    Cf*lf/Iz
];

C = [
    Cr/m/Vx
    -Cr*lr/Iz
];

tmp = inv([B C]) * A