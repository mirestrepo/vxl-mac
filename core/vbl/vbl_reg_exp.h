// Original Copyright notice:
// Copyright (C) 1991 Texas Instruments Incorporated.
//
// Permission is granted to any individual or institution to use, copy, modify,
// and distribute this software, provided that this complete copyright and
// permission notice is maintained, intact, in all copies and supporting
// documentation.
//
// Texas Instruments Incorporated provides this software "as is" without
// express or implied warranty.
//
// .LIBRARY vbl
// .HEADER vxl package
// .INCLUDE vbl/vbl_reg_exp.h
// .FILE vbl_reg_exp.cxx
// .EXAMPLE examples/vbl_reg_exp_example.cxx
//
#ifndef vbl_reg_exph
#define vbl_reg_exph

#include <vcl/vcl_string.h>

const int NSUBEXP = 10;

//: Pattern matching with regular expressions
//  A regular expression allows a programmer to specify  complex
//  patterns  that  can  be searched for and matched against the
//  character string of a string object. In its simplest form, a
//  regular  expression  is  a  sequence  of  characters used to
//  search for exact character matches. However, many times  the
//  exact  sequence to be found is not known, or only a match at
//  the beginning or end of a string is desired. The vbl  regu-
//  lar  expression  class implements regular expression pattern
//  matching as is found and implemented in many  UNIX  commands
//  and utilities.
//
//  Example: The perl code
//  
//     $filename =~ m"([a-z]+)\.cc";
//     print $1;
//     
//  Is written as follows in C++
//
//     vbl_reg_exp re("([a-z]+)\\.cc");
//     re.find(filename);
//     cerr << re.match(1);
//
//
//  The regular expression class provides a convenient mechanism
//  for  specifying  and  manipulating  regular expressions. The
//  regular expression object allows specification of such  pat-
//  terns  by using the following regular expression metacharac-
//  ters:
// 
//   ^        Matches at beginning of a line
//
//   $        Matches at end of a line
//
//  .         Matches any single character
//
//  [ ]       Matches any character(s) inside the brackets
//
//  [^ ]      Matches any character(s) not inside the brackets
//
//   -        Matches any character in range on either side of a dash
//
//   *        Matches preceding pattern zero or more times
//
//   +        Matches preceding pattern one or more times
//
//   ?        Matches preceding pattern zero or once only
//
//  ()        Saves a matched expression and uses it in a  later match
// 
//  Note that more than one of these metacharacters can be  used
//  in  a  single  regular expression in order to create complex
//  search patterns. For example, the pattern [^ab1-9]  says  to
//  match  any  character  sequence that does not begin with the
//  characters "ab"  followed  by  numbers  in  the  series  one
//  through nine.
//
class vbl_reg_exp {
public:
  inline vbl_reg_exp ();			// vbl_reg_exp with program=NULL
  inline vbl_reg_exp (char const*);	// vbl_reg_exp with compiled char*
  vbl_reg_exp (vbl_reg_exp const&);	// Copy constructor
  inline ~vbl_reg_exp();			// Destructor 

  void compile (char const*);		// Compiles char* --> regexp
  bool find (char const*);		// true if regexp in char* arg
  bool find (vcl_string const&);		// true if regexp in char* arg
  inline long start() const;		// Index to start of first find
  inline long end() const;		// Index to end of first find

  bool operator== (vbl_reg_exp const&) const;	// Equality operator
  inline bool operator!= (vbl_reg_exp const&) const; // Inequality operator
  bool deep_equal (vbl_reg_exp const&) const;	// Same regexp and state?
  
  inline bool is_valid() const;		// true if compiled regexp
  inline void set_invalid();		// Invalidates regexp

  // awf added
  int start(int n) const;
  int end(int n) const;
  vcl_string match(int n) const;
  
private: 
  const char* startp[NSUBEXP];
  const char* endp[NSUBEXP];
  char  regstart;			// Internal use only
  char  reganch;			// Internal use only
  const char* regmust;			// Internal use only
  int   regmlen;			// Internal use only
  char* program;   
  int   progsize;
  const char* searchstring;
}; 

// -- Creates an empty regular expression.

inline vbl_reg_exp::vbl_reg_exp () { 
  this->program = NULL;
}


// -- Creates a regular expression from string s, and compiles s.

inline vbl_reg_exp::vbl_reg_exp (const char* s) {  
  this->program = NULL;
  compile(s);
}

// -- Frees space allocated for regular expression.

inline vbl_reg_exp::~vbl_reg_exp () {
//#ifndef WIN32
  delete [] this->program;
//#endif
}

// -- 

inline long vbl_reg_exp::start () const {
  return(this->startp[0] - searchstring);
}


// -- Returns the start/end index of the last item found.

inline long vbl_reg_exp::end () const {
  return(this->endp[0] - searchstring);
}


// operator!= 

inline bool vbl_reg_exp::operator!= (const vbl_reg_exp& r) const {
  return(!(*this == r));
}


// -- Returns true if a valid regular expression is compiled and ready for pattern matching.

inline bool vbl_reg_exp::is_valid () const {
  return (this->program != NULL);
}


// -- Invalidates regular expression.

inline void vbl_reg_exp::set_invalid () {
//#ifndef WIN32
  delete [] this->program;
//#endif
  this->program = NULL;
}

// -- Return start index of nth submatch. start(0) is the start of the full match.

inline int vbl_reg_exp::start(int n) const
{
  return this->startp[n] - searchstring;
}

// -- Return end index of nth submatch. end(0) is the end of the full match.

inline int vbl_reg_exp::end(int n) const
{
  return this->endp[n] - searchstring;
}

// -- Return nth submatch as a string.

inline vcl_string vbl_reg_exp::match(int n) const
{
  return vcl_string(this->startp[n], this->endp[n] - this->startp[n]);
}

#endif // vbl_reg_exph
