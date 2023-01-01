Scalar const c0 = pow(omega[0], 2);
Scalar const c1 = pow(omega[1], 2);
Scalar const c2 = pow(omega[2], 2);
Scalar const c3 = c0 + c1 + c2;
Scalar const c4 = sqrt(c3);
Scalar const c5 = 0.5 * c4;
Scalar const c6 = sin(c5);
Scalar const c7 = c6 / c4;
Scalar const c8 = c6 / pow(c3, 3.0 / 2.0);
Scalar const c9 = 0.5 * cos(c5) / c3;
Scalar const c10 = c8 * omega[0];
Scalar const c11 = c9 * omega[0];
Scalar const c12 = -c10 * omega[1] + c11 * omega[1];
Scalar const c13 = -c10 * omega[2] + c11 * omega[2];
Scalar const c14 = omega[1] * omega[2];
Scalar const c15 = -c14 * c8 + c14 * c9;
Scalar const c16 = 0.5 * c7;
result[0] = -
c0 *c8
+
c0 *c9
+
c7;
result[1] =
c12;
result[2] =
c13;
result[3] =
c12;
result[4] = -
c1 *c8
+
c1 *c9
+
c7;
result[5] =
c15;
result[6] =
c13;
result[7] =
c15;
result[8] = -
c2 *c8
+
c2 *c9
+
c7;
result[9] = -
c16 *omega[0];
result[10] = -
c16 *omega[1];
result[11] = -
c16 *omega[2];
