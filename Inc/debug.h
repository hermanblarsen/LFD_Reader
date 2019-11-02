#ifndef _DEBUG_H_
#define _DEBUG_H_

/*=============================================================================
 *   This header file adds optional compilation of debug print statements and definitions
 *    used in this project to reduce final compile size of the project while still being
 *    able to optionally add in this functionality by definging one of two constants.
 *   Two debug modes can be defined using -D{DEBUG_SOFT|DEBUG_HARD} in the
 *    compiler command line 'misc' options.
 *   The two definitions will result in the following:
 *       DEBUG_SOFT
 *           will enable the DPRINT commands used in the project
 *           will enable the DEBUG_ASSERT(X) statement that sets a breakpoint if !x
 *       DEBUG_HARD
 *           will enable everything DEBUG_SOFT does above
 *           will also remove the "static" keyword from some key variables to allow
 *            monitoring outside their translation unit in debugging sessions.
 *   Not defining a debug mode results in DPRINT or DEBUG_ASSERT statements being
 *    removed in the preprocessing step to reduce build size on the final build.
 *   Due to the 'intrusive' nature of DEBUG_HARD (some static variables are set to
 *    non-static, only using this when absolutely necessary is recommended.
=============================================================================*/
#define DEBUG_SOFT
//#define DEBUG_HARD
/*=============================================================================
**       Definitions
=============================================================================*/
/*  If any DEBUG mode is defined,
     define the single line DPRINT and DPRINTS statements.
    Also define DEBUG which can be used for checking if in any debug mode. */
#if defined (DEBUG_SOFT) || defined (DEBUG_HARD)
#define DEBUG
# include <stdio.h>

/* DPRINTS prints as printf(); */
# define DPRINTS(...) printf(__VA_ARGS__)
/* DPRINT prints as printf();, but also prints the file and line number
    prior to the printf() content */
# define DPRINT(...) do{D1PRINT(); D2PRINT(__VA_ARGS__);}while(0)
/* Internal macro definitions for above dprint statements */
# define D1PRINT() printf("%s %d DEBUG: ", __FILE__, __LINE__);
# define D2PRINT(...) printf(__VA_ARGS__)

/* DEBUG_ASSERT Breakpoint statement , will if x == 0*/
# define DEBUG_ASSERT(x) do{if(!(x))__breakpoint(0);}while(0)

#else  /*If not in a DEBUG mode, define as nothing */
# define DPRINTS(...)
# define DPRINT(...)
# define D1PRINT(...)
# define D2PRINT(...)
# define DEBUG_ASSERT(x)
#endif

/*  THIS FUNCTIONALITY SHOULD NOT BE USED, AND IS COMMENTED OUT.
    In DEBUG_HARD redefine static variables to non-static for global watchability.
    This is very intrusive and should not be used. A better alternative is redefining
     some required specific variables to non-static surrounded by
    #ifndef DEBUG_HARD
     static Type variable;
    #else
     Type variable;
    #endif
*/
//#ifdef DEBUG_HARD
//    /* Redefine 'static' to '' so that all static variables are non-static */
//# define static
//#endif

#endif /* _DEBUG_H_ */
