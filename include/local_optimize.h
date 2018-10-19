//
// a per function optimization level for use with __attribute__( ( optimize( "O0" ) ) )
// statements embedded in the code
//

#ifndef __LOCAL_OPTIMIZE_H__
#define __LOCAL_OPTIMIZE_H__

//#define __LOCAL_DEBUG
#undef __LOCAL_DEBUG

#define LOCAL_OPTIMZATION_LEVEL      "O0"

#ifdef __LOCAL_DEBUG
#define __SET_LOCAL_OPTIMIZATION_LEVEL      __attribute__( ( optimize( LOCAL_OPTIMZATION_LEVEL ) ) )
#else
#define __SET_LOCAL_OPTIMIZATION_LEVEL
#endif

#endif
