#define NZ  339 //! Total number of prefixes
#define NZ2 12  //! Total number of suffixes
// character*1 sfx(NZ2)
// character*5 pfx(NZ)

#include "mshv_support.h"

extern char sfx[NZ2];
extern  QString pfx[NZ];

void init_pfx_sfx();
