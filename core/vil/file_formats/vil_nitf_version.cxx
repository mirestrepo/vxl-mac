// $Source$

//
// Description:	Implementation of for class vil_nitf_version.
//
// $Revision$ 
// $Date$
// $Author$
//

#include <vcl_cstring.h>

#include "vil_nitf_macro_defs.h"
#include "vil_nitf_version.h"

//====================================================================
// Constructor(s) and destructor.  Note: the constructor is protected.
//====================================================================
vil_nitf_version::vil_nitf_version (const char* name)
{
    uvers = 0 ;
    cvers = 0 ;
    STRCPY (cvers, name ? name : "") ;
}

vil_nitf_version::~vil_nitf_version()
{
    delete cvers ;
}

//====================================================================
// Operator== can be used to determine if two NITF objects are the
// same version.
//====================================================================
bool vil_nitf_version::operator== (const vil_nitf_version& otherVersion)
{
    bool rval = false;

    if (cvers != 0 && otherVersion.cvers != 0) {
      if ((cvers == otherVersion.cvers) 
	  || strcmp (cvers, otherVersion.cvers) == 0) {
	rval = true ;
      }
    }
    return rval ;
}

