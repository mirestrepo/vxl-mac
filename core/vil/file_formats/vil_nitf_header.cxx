// This is core/vil/file_formats/vil_nitf_header.cxx
#include "vil_nitf_header.h"
//================ GE Aerospace NITF support libraries =================
//:
// \file
// \date: 2003/12/26
// \author: mlaymon
//
// \brief Implementation of headers included in an NITFS message.
// See vil_nitf_header.h for details.
//
// Written by:       Burt Smith
// Date:             August 26, 1992
//
// Modification by:  Paul Max Payton (PMP)
// Date:             August 14, 1997
// Why?              Added rational polynomial stuff for NITF's image subheader.
//
// Modification by:  Glen W. Brooksby (GWB)
// Date:             February 18, 1999
// Why?:             Added support for ICHIPB Support Data Extension
//
//=====================lkbjfcbtdddhtargbskmtaps=======================
//
// Copyright (C) 1998, Lockheed Martin Corporation
//
// This software is intellectual property of Lockheed Martin
// Corporation and may not be copied or redistributed except
// as specified in the FOCUS Software License.
//

#include "vil_nitf_macro_defs.h"
#include "vil_nitf_version.h"
#include "vil_nitf_util.h"

//====================================================================
// Generic NITF Header object.
//====================================================================

vil_nitf_header::vil_nitf_header(unsigned long len)
  : reference_count_( 0 )
{
    data_length_ = len > 0 ? len : 0;
    ID = DT = TITLE = 0;

    STRCPY(ID, "");
    STRCPY(DT, "");
    STRCPY(TITLE, "");

    CLAS = DefaultClassification;
    CODE = CTLH = REL = CAUT = CTLN = DWNG = DEVT = 0;
    ENCRYP = NOTENCRYPTED;

    STRCPY(CODE,"");
    STRCPY(CTLH,"");
    STRCPY(REL,"");
    STRCPY(CAUT,"");
    STRCPY(CTLN,"");
    STRCPY(DWNG,"");
    STRCPY(DEVT,"");

    DLVL = 1;
    ALVL = LOCrow = LOCcolumn = 0;

    version_ = "";

#ifdef TRACE
    verbose_ = true;
#else
    verbose_ = false;
#endif
}

//====================================================================
//: Copy constructor for vil_nitf_header.
//====================================================================
vil_nitf_header::vil_nitf_header(const vil_nitf_header& header)
{
  ID    = new_strdup(header.ID);
  DT    = new_strdup(header.DT);
  TITLE = new_strdup(header.TITLE);

  CLAS  = header.CLAS;

  CODE  = new_strdup(header.CODE);
  CTLH  = new_strdup(header.CTLH);
  REL   = new_strdup(header.REL);
  CAUT  = new_strdup(header.CAUT);
  CTLN  = new_strdup(header.CTLN);
  DWNG  = new_strdup(header.DWNG);
  DEVT  = new_strdup(header.DEVT);

  ENCRYP    = header.ENCRYP;
  DLVL      = header.DLVL;
  ALVL      = header.ALVL;
  LOCrow    = header.LOCrow;
  LOCcolumn = header.LOCcolumn;

  version_ = "";
}

vil_nitf_header::~vil_nitf_header()
{
  delete[] ID;
  delete[] DT;
  delete[] TITLE;
  delete[] CODE;
  delete[] CTLH;
  delete[] REL;
  delete[] CAUT;
  delete[] CTLN;
  delete[] DWNG;
  delete[] DEVT;

#if 0 // COMMENT OUT UNTIL I DECIDE WHAT TO DO ABOUT NITFVersion
  if (version_) version_->unref();
#endif
}

void vil_nitf_header::setVersion(const vcl_string & v)
{
  version_ = v;
}

//====================================================================
//: Method to return a copy of the vil_nitf_header.
//  The copy returned *must* be deleted by the caller.
//====================================================================
vil_nitf_header* vil_nitf_header::Copy()
{
  vil_nitf_header* rval = new vil_nitf_header(*this);
  return rval;
}

void vil_nitf_header::Copy(const vil_nitf_header* h)
{
  data_length_ = h->data_length_;

  FilledCopy(ID, h->ID);
  // FilledCopy(DT, h->DT);
  FilledCopy(TITLE, h->TITLE);

  CLAS = h->CLAS;

  FilledCopy(CODE, h->CODE);
  FilledCopy(CTLH, h->CTLH);
  FilledCopy(REL, h->REL);
  FilledCopy(CAUT, h->CAUT);
  FilledCopy(CTLN, h->CTLN);
  FilledCopy(DWNG, h->DWNG);
  FilledCopy(DEVT, h->DEVT);

  ENCRYP = h->ENCRYP;
  DLVL   = h->DLVL;
  ALVL   = h->ALVL;
  LOCrow = h->LOCrow;
  LOCcolumn = h->LOCcolumn;
}


void vil_nitf_header::set_title(char * new_val)
{
    FilledCopy(TITLE, new_val);
}

//====================================================================
// Stuff to deal with classifications.
//====================================================================

NITFClass DefaultClassification = UNCLASSIFIED;

NITFClass ConvertClassification(const char* inc)
{
  NITFClass c = DefaultClassification;

  char* cc = 0;
  STRCPY(cc, inc ? inc : "");
  squeeze(cc, ' ', 0);

  // NOTE:  FIGURE WHAT vcl_XXX METHOD TO USE FOR nitf_strcasecmp.  MAL 3oct2003
  if (!nitf_strcasecmp(cc,"UNCLASSIFIED") || !nitf_strcasecmp(cc,"U"))
    c = UNCLASSIFIED;
  else if (!nitf_strcasecmp(cc, "RESTRICTED") || !nitf_strcasecmp(cc,"R"))
    c = RESTRICTED;
  else if (!nitf_strcasecmp(cc, "CONFIDENTIAL") || !nitf_strcasecmp(cc,"C"))
    c = CONFIDENTIAL;
  else if (!nitf_strcasecmp(cc, "SECRET") || !nitf_strcasecmp(cc, "S"))
    c = SECRET;
  else if (!nitf_strcasecmp(cc, "TOPSECRET") || !nitf_strcasecmp(cc, "T"))
    c = TOPSECRET;

  // MPP 5/7/2002
  // Plugging memory leaks
  delete[] cc;

  return c;
}

const char * ConvertClassification(NITFClass c)
{
  static char* cc = 0;

  if (!(c==UNCLASSIFIED || c==RESTRICTED || c==CONFIDENTIAL ||
        c==SECRET       || c==TOPSECRET))
    c = DefaultClassification;

  switch(c)
  {
    case UNCLASSIFIED: STRCPY(cc,"UNCLASSIFIED"); break;
    case RESTRICTED:   STRCPY(cc,"RESTRICTED");   break;
    case CONFIDENTIAL: STRCPY(cc,"CONFIDENTIAL"); break;
    case SECRET:       STRCPY(cc,"SECRET");       break;
    case TOPSECRET:    STRCPY(cc,"TOPSECRET");    break;
  }
  return cc;
}

bool ValidClassification(const char* inc)
{
  bool rval = false;
  char* cc = 0; // TEMPORARY char ARRAY.  DELETE WHEN DONE !

  STRCPY(cc, inc?inc:"");
  squeeze(cc, ' ', 0);

  if (!nitf_strcasecmp(cc, "UNCLASSIFIED") || !nitf_strcasecmp(cc, "U") ||
      !nitf_strcasecmp(cc, "RESTRICTED")   || !nitf_strcasecmp(cc, "R") ||
      !nitf_strcasecmp(cc, "CONFIDENTIAL") || !nitf_strcasecmp(cc, "C") ||
      !nitf_strcasecmp(cc, "SECRET")       || !nitf_strcasecmp(cc, "S") ||
      !nitf_strcasecmp(cc, "TOPSECRET")    || !nitf_strcasecmp(cc, "T"))
    rval = true;

  // MPP 5/7/2002
  // Plugging memory leaks
  delete[] cc;

  return rval;
}
