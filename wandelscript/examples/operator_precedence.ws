# Experiments around operator precedence, which is a bit different in W'script
# compared to Python.

a0  = 1 + 2 * 3
a1  = 1 * 2 + 3
a2  = 3 * 4 / 2
a3  = 4 / 2 * 3
a4  = 1 + 4 / 2
a5  = 4 / 2 + 1
a6  = +2 - 1
a7  = -1 + +2
a8  = -1 + -2
a9  = ~1 + 1
a10 =  1 + ~1
a11 =  -1 * 2
a12 =  1 or 2 + 3
a13 =  2 + 3 or 1
a14 =  1 or 2 * 3
a15 =  2 * 3 or 1
a16 =  1 and 2 + 3
a17 =  2 + 3 and 1
a18 =  1 and 2 * 3
a19 =  2 * 3 and 1
a20 =  1 or 2 and 3  # ws 3 py 1
a21 =  1 and 2 or 3
a22 =  (1 or 2) and 3
a23 =  1 or (2 and 3)
a24 =  (1 and 2) or 3
a25 =  1 and (2 or 3)
a26 =  1 and 3 > 2
a27 =  1 or 3 > 2
a28 =  2 > 3 and 1
a29 =  1 > 3 or 2
a30 =  not 1 and 2
a31 =  1 and not 0
a32 =  2 or not 0
a33 =  not 0 or 2
a34 =  2 and not 0
a35 =  not 0 and 2
a36 =  2 and 3 == 3
a37 =  2 and 3 == 4
a38 =  3 == 3 and 2
a39 =  3 == 4 and 2
a40 =  2 or 3 == 3
a41 =  2 or 3 == 4
a42 =  3 == 3 or 2
a43 =  3 == 4 or 2
a44 =  3 < 4 < 5
a45 =  5 > 4 > 3  # ws False py True; bc ternary in Py
a46 =  5 > 4 < 3  # ws true py False; bc ternary in Py
a47 =  (5 > 4) == True
a48 =  5 > 4 == True  # ws True py False
a49 =  5 < 4 == True
a50 =  not False == True
a51 =  not 3 == True  # ws False py True
a52 =  not 3 < True
a53 =  3 > (not 2)
a54 =  True > not False  # ws False Py syntax error
a55 =  4 < 3 or 2
a56 = 1 + 2 * 3
a57 = 1 + 2 and 3 + 4
a58 = 1 + 2 or 3 + 4
a59 = 3 * 2 + 1
a60 = not 3 + 4
a61 = not 3 * 4
a62 = not 3 * not 4
a63 = 3 * not 0
a64 = not -3
a65 = not ~ -1
a66 = not (~(-(1)))
a67 = (1 + 2) * ((3 + 5 * 2) / (4 + 2))
