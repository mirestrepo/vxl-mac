#ifdef __GNUC__
#pragma implemantation
#endif

//:
// \file
// \brief Asks question and waits for an answer
// \author tim
// hand crafted into vxl by gvw
//
//  Function Name:  mbl_read_double
//  Synopsis:       double mbl_read_double(char* q_str, double default_d)
//  Inputs:         q_str: A question
//                  default_d: Default answer
//                  min_d: Min allowed value (optional)
//                  max_d: Max allowed value (optional)
//  Outputs:        -
//  Returns:        The answer or a default
//  Description:    Asks question and waits for an answer.
//                  If the answer is a double, returns it.
//                  If the answer is an empty vcl_string (return)
//                  then returns default.
//                  Otherwise waits for another input.
//  References:     -
//  Example:
//    double new_scale = mbl_read_double("Scale?",1.00);
//    double new_scale = mbl_read_double("Scale?",1.00,min_scale,max_scale);

#include <mbl/mbl_read_double.h>

const int MAX_LEN = 40;

// If min_d != 0 or max_d != 0 then prints range but doesn't check that reply is in range
double RD_ReadDouble1(char *q_str, double default_d,
                      double min_d, double max_d)
{
  char reply[MAX_LEN];

  while (true)
  {
    if (min_d==0 && max_d==0)
      vcl_cout<<q_str<<" ("<<default_d<<") :";
    else
      vcl_cout<<q_str<<" ["<<min_d<<".."<<max_d<<"] ("<<default_d<<") :";
    vcl_cout.flush();

    if (fgets(reply,MAX_LEN,stdin)!=NULL)
    {
      double r = default_d;
      if (reply[0]=='\n' || sscanf(reply,"%lf",&r)>0)
        return r;
    }
  }
}

double mbl_read_double(char *q_str, double default_d)
{
  return RD_ReadDouble1(q_str,default_d,0,0);
}

double mbl_read_double( char *q_str, double default_d,
                        double min_d, double max_d)
{
  while (true)
  {
    double R = RD_ReadDouble1(q_str,default_d,min_d,max_d);
    if (R<min_d)
      vcl_cout<<R<<": must be at least "<<min_d<<"\n";
    else if (R>max_d)
      vcl_cout<<R<<": must be no more than "<<max_d<<"\n";
    else
      return R; // acceptable
  }
}

