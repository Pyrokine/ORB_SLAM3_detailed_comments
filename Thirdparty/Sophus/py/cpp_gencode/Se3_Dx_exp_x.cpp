Scalar const c0 = pow(omega[0], 2);
Scalar const c1 = pow(omega[1], 2);
Scalar const c2 = pow(omega[2], 2);
Scalar const c3 = c0 + c1 + c2;
Scalar const c4 = sqrt(c3);
Scalar const c5 = 1.0 / c4;
Scalar const c6 = 0.5 * c4;
Scalar const c7 = sin(c6);
Scalar const c8 = c5 * c7;
Scalar const c9 = pow(c3, -3.0 / 2.0);
Scalar const c10 = c7 * c9;
Scalar const c11 = 1.0 / c3;
Scalar const c12 = 0.5 * c11 * cos(c6);
Scalar const c13 = c10 * omega[0];
Scalar const c14 = omega[0] * omega[1];
Scalar const c15 = c12 * c14 - c13 * omega[1];
Scalar const c16 = omega[0] * omega[2];
Scalar const c17 = c12 * c16 - c13 * omega[2];
Scalar const c18 = omega[1] * omega[2];
Scalar const c19 = -c10 * c18 + c12 * c18;
Scalar const c20 = 0.5 * c8;
Scalar const c21 = sin(c4);
Scalar const c22 = -c21 + c4;
Scalar const c23 = -c1;
Scalar const c24 = -c2;
Scalar const c25 = c23 + c24;
Scalar const c26 = c25 * c9;
Scalar const c27 = cos(c4);
Scalar const c28 = 1 - c27;
Scalar const c29 = c11 * c28;
Scalar const c30 = c29 * omega[2];
Scalar const c31 = c22 * c9;
Scalar const c32 = c31 * omega[1];
Scalar const c33 = c32 * omega[0];
Scalar const c34 = c29 * omega[1];
Scalar const c35 = c31 * omega[2];
Scalar const c36 = c35 * omega[0];
Scalar const c37 = 3 * c22 / pow(c3, 5.0 / 2.0);
Scalar const c38 = c25 * c37;
Scalar const c39 = c5 * omega[0];
Scalar const c40 = -c27 * c39 + c39;
Scalar const c41 = c40 * c9;
Scalar const c42 = c0 * c37;
Scalar const c43 = c16 * c41 + c35 - c42 * omega[2];
Scalar const c44 = c21 * c9;
Scalar const c45 = c14 * c44;
Scalar const c46 = 2 * c28 / pow(c3, 2);
Scalar const c47 = c14 * c46;
Scalar const c48 = c45 - c47;
Scalar const c49 = c14 * c41 + c32 - c42 * omega[1];
Scalar const c50 = c16 * c44;
Scalar const c51 = c16 * c46;
Scalar const c52 = -c50 + c51;
Scalar const c53 = -2 * c32;
Scalar const c54 = c5 * omega[1];
Scalar const c55 = -c27 * c54 + c54;
Scalar const c56 = c1 * c44;
Scalar const c57 = c1 * c46;
Scalar const c58 = c55 * c9;
Scalar const c59 = c16 * c58;
Scalar const c60 = c37 * omega[0];
Scalar const c61 = -c18 * c60;
Scalar const c62 = c29 + c61;
Scalar const c63 = c31 * omega[0];
Scalar const c64 = -c1 * c60 + c14 * c58 + c63;
Scalar const c65 = c18 * c44;
Scalar const c66 = c18 * c46;
Scalar const c67 = -c65 + c66;
Scalar const c68 = -2 * c35;
Scalar const c69 = c5 * omega[2];
Scalar const c70 = -c27 * c69 + c69;
Scalar const c71 = c2 * c44;
Scalar const c72 = c2 * c46;
Scalar const c73 = c70 * c9;
Scalar const c74 = c14 * c73;
Scalar const c75 = -c29 + c61;
Scalar const c76 = c65 - c66;
Scalar const c77 = c16 * c73 - c2 * c60 + c63;
Scalar const c78 = -c0;
Scalar const c79 = c24 + c78;
Scalar const c80 = c29 * omega[0];
Scalar const c81 = c32 * omega[2];
Scalar const c82 = -2 * c63;
Scalar const c83 = c79 * c9;
Scalar const c84 = c0 * c44;
Scalar const c85 = c0 * c46;
Scalar const c86 = c18 * c41;
Scalar const c87 = c50 - c51;
Scalar const c88 = c37 * c79;
Scalar const c89 = -c45 + c47;
Scalar const c90 = c37 * omega[2];
Scalar const c91 = -c1 * c90 + c18 * c58 + c35;
Scalar const c92 = c37 * omega[1];
Scalar const c93 = c18 * c73 - c2 * c92 + c32;
Scalar const c94 = c23 + c78;
Scalar const c95 = c9 * c94;
result[0] = 0;
result[1] = 0;
result[2] = 0;
result[3] = -
c0 *c10
+
c0 *c12
+
c8;
result[4] =
c15;
result[5] =
c17;
result[6] = 0;
result[7] = 0;
result[8] = 0;
result[9] =
c15;
result[10] = -
c1 *c10
+
c1 *c12
+
c8;
result[11] =
c19;
result[12] = 0;
result[13] = 0;
result[14] = 0;
result[15] =
c17;
result[16] =
c19;
result[17] = -
c10 *c2
+
c12 *c2
+
c8;
result[18] = 0;
result[19] = 0;
result[20] = 0;
result[21] = -
c20 *omega[0];
result[22] = -
c20 *omega[1];
result[23] = -
c20 *omega[2];
result[24] =
c22 *c26
+ 1;
result[25] = -c30 +
c33;
result[26] = c34 +
c36;
result[27] = upsilon[0]*(
c26 *c40
-
c38 *omega[0]
) + upsilon[1]*(c49 + c52) + upsilon[2]*(c43 + c48);
result[28] = upsilon[0]*(
c26 *c55
-
c38 *omega[1]
+ c53) + upsilon[1]*(c64 + c67) + upsilon[2]*(c56 - c57 + c59 + c62);
result[29] = upsilon[0]*(
c26 *c70
-
c38 *omega[2]
+ c68) + upsilon[1]*(-c71 + c72 + c74 + c75) + upsilon[2]*(c76 + c77);
result[30] = c30 +
c33;
result[31] =
c31 *c79
+ 1;
result[32] = -c80 +
c81;
result[33] = upsilon[0]*(c49 + c87) + upsilon[1]*(
c40 *c83
-
c60 *c79
+ c82) + upsilon[2]*(c75 - c84 + c85 + c86);
result[34] = upsilon[0]*(c64 + c76) + upsilon[1]*(
c55 *c83
-
c88 *omega[1]
) + upsilon[2]*(c89 + c91);
result[35] = upsilon[0]*(c62 + c71 - c72 + c74) + upsilon[1]*(c68 +
c70 *c83
-
c88 *omega[2]
) + upsilon[2]*(c52 + c93);
result[36] = -c34 +
c36;
result[37] = c80 +
c81;
result[38] =
c31 *c94
+ 1;
result[39] = upsilon[0]*(c43 + c89) + upsilon[1]*(c62 + c84 - c85 + c86) + upsilon[2]*(
c40 *c95
-
c60 *c94
+ c82);
result[40] = upsilon[0]*(-c56 + c57 + c59 + c75) + upsilon[1]*(c48 + c91) + upsilon[2]*(c53 +
c55 *c95
-
c92 *c94
);
result[41] = upsilon[0]*(c67 + c77) + upsilon[1]*(c87 + c93) + upsilon[2]*(
c70 *c95
-
c90 *c94
);
