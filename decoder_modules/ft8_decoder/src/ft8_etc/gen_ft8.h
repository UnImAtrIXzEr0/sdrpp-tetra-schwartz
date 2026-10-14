#ifndef GEN_FT8_H
#define GEN_FT8_H

#include "mshv_support.h"
//#include <QObject>


//#include <QApplication>
//#include <QCoreApplication>
//#include <stdio.h>      /* printf */
//#include <math.h>       /* fmod */

// JTMSK144 ///////////////////////
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>

#include "pack_unpack_msg77.h"
#include "genpom.h"
// #include "gengenpom.h"
//#include <QStringList>


//#include <complex.h> // gnu++11 c++11
//#define complex		_Complex
class GenFt8 //: public QObject
{
	//Q_OBJECT // hv  
public: 
    explicit GenFt8(bool fl);//f_dec_gen = dec=true gen=false
    ~GenFt8();
 
    int genft8(QString,short *t_iwave,double samp_rate,double f0);//,int i3b ,int &ntxslot 
    void make_c77_i4tone(bool *c77,int *i4tone); 
    void make_c77_i4tone_codeword(bool *c77,int *i4tone,bool*);                          
    QString GetUnpackMsg(){return s_unpack_msg;};
    void save_hash_call_from_dec(QString c13,int n10,int n12,int n22);
    void save_hash_call_my_his_r1_r2(QString call,int pos);
    //void save_hash_call_mam(QStringList ls);
    QString unpack77(bool *c77,bool &unpk77_success); 
    void pack77(QString msgs,int &i3,int n3,bool *c77); 
    void split77(QString &msg,int &nwords,/*int *nw,*/QString *w); 
    	
private:   
    GenPomFt genPomFt;      
    PackUnpackMsg77 TPackUnpackMsg77;
    double twopi;
    //QString format_msg(char *message_in, int cmsg);
    //void make_c77_i4tone(bool *c77,int *i4tone);//,bool f_gen,bool f_addc
public:
    QString s_unpack_msg;

};
#endif