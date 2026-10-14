/* The algorithms, source code, look-and-feel of WSJT-X and related
 * programs, and protocol specifications for the modes FSK441, FT8, JT4,
 * JT6M, JT9, JT65, JTMS, QRA64, ISCAT, MSK144, are Copyright © 2001-2017
 * by one or more of the following authors: Joseph Taylor, K1JT; Bill
 * Somerville, G4WJS; Steven Franke, K9AN; Nico Palermo, IV3NWV; Greg Beam,
 * KI7MT; Michael Black, W9MDB; Edson Pereira, PY2SDR; Philip Karn, KA9Q;
 * and other members of the WSJT Development Group.
 *
 * MSHV Decoder/Generator
 * Rewritten into C++ and modified by Hrisimir Hristov, LZ2HV 2015-2022
 * May be used under the terms of the GNU General Public License (GPL)
 */
#ifndef GENPOM_H
#define GENPOM_H
#include <math.h>

double gfsk_pulse_(double b,double t);
void gen_pulse_gfsk_(double *pulse,double k,double bt,int nsps);

class GenPomFt
{
public:
    void initGenPomFt();
    void encode174_91(bool *message77,bool *codeword);
private:
    bool first_ft_enc_174_91;
    char genft_174_91[100][95];//91 83
    short crc14(unsigned char const * data, int length);
};


#endif