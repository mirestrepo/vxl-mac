#ifndef mbl_read_double_h_
#define mbl_read_double_h_

#ifdef __GNUC__
#pragma interface
#endif

//:
// \file
// \brief Description!
// \author tim
// hand crafted into vxl by gvw
// 
//  Function Name:  mbl_read_double
//  Synopsis: double mbl_read_double(char* q_str, double default_d)
//  Inputs:   q_str: A question 
//      default_d: Default answer
//        min_d: Min allowed value (optional)
//        max_d: Max allowed reply (optional)
//  Outputs:  -
//  Returns:    The answer or a default
//  Description:  Asks question and waits for an answer.
//      If the answer is a double, returns it.
//      If the answer is an empty vcl_string (return)
//      then returns default.
//      Otherwise waits for another input.
//  References: -
//  Example: 
//    new_scale = mbl_read_double("Scale?",1.00);
//    double new_scale = mbl_read_double("Scale?",1.00,min_scale,max_scale);

#include <vcl_iostream.h>

double mbl_read_double(char* q_str, double default_d);
double mbl_read_double(char* q_str, double default_d,
          double min_d, double max_d);

#endif //mbl_read_double_h_
