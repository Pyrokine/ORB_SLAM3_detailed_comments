Scalar const c0 = 0.5 * q.w();
Scalar const c1 = 0.5 * q.z();
Scalar const c2 = -c1;
Scalar const c3 = 0.5 * q.y();
Scalar const c4 = 0.5 * q.x();
Scalar const c5 = -c4;
Scalar const c6 = -c3;
Scalar const c7 = pow(q.x(), 2);
Scalar const c8 = pow(q.y(), 2);
Scalar const c9 = -c8;
Scalar const c10 = pow(q.w(), 2);
Scalar const c11 = pow(q.z(), 2);
Scalar const c12 = c10 - c11;
Scalar const c13 = 2 * q.w();
Scalar const c14 = c13 * q.z();
Scalar const c15 = 2 * q.x();
Scalar const c16 = c15 * q.y();
Scalar const c17 = c13 * q.y();
Scalar const c18 = c15 * q.z();
Scalar const c19 = -c7;
Scalar const c20 = c13 * q.x();
Scalar const c21 = 2 * q.y() * q.z();
result[0] = 0;
result[1] = 0;
result[2] = 0;
result[3] =
c0;
result[4] =
c2;
result[5] =
c3;
result[6] = 0;
result[7] = 0;
result[8] = 0;
result[9] =
c1;
result[10] =
c0;
result[11] =
c5;
result[12] = 0;
result[13] = 0;
result[14] = 0;
result[15] =
c6;
result[16] =
c4;
result[17] =
c0;
result[18] = 0;
result[19] = 0;
result[20] = 0;
result[21] =
c5;
result[22] =
c6;
result[23] =
c2;
result[24] = c12 + c7 +
c9;
result[25] = -c14 +
c16;
result[26] = c17 +
c18;
result[27] = 0;
result[28] = 0;
result[29] = 0;
result[30] = c14 +
c16;
result[31] = c12 + c19 +
c8;
result[32] = -c20 +
c21;
result[33] = 0;
result[34] = 0;
result[35] = 0;
result[36] = -c17 +
c18;
result[37] = c20 +
c21;
result[38] = c10 + c11 + c19 +
c9;
result[39] = 0;
result[40] = 0;
result[41] = 0;
