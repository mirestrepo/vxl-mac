// This is brl/bseg/brec/pro/brec_prob_map_supress_process.h
#ifndef brec_prob_map_supress_process_h_
#define brec_prob_map_supress_process_h_
//:
// \file
// \brief A class to suppress a given density/prob map with respect to another map, e.g. to remove vehicle areas from a change map
//
// Input map is P(X in B), suppressor is e.g. P(X in V) (x is a vehicles pixel)
// Output of this process is: P(X in B and X not in V)
//
// \author Ozge Can Ozcanli
// \date 10/28/08
//
// \verbatim
//  Modifications
//   <none yet>
// \endverbatim

#include <vcl_string.h>
#include <bprb/bprb_process.h>

class brec_prob_map_supress_process : public bprb_process
{
 public:

  brec_prob_map_supress_process();

  //: Copy Constructor (no local data)
  brec_prob_map_supress_process(const brec_prob_map_supress_process& other): bprb_process(*static_cast<const bprb_process*>(&other)){};

  ~brec_prob_map_supress_process(){};

  //: Clone the process
  virtual brec_prob_map_supress_process* clone() const {return new brec_prob_map_supress_process(*this);}

  vcl_string name() { return "brecProbMapSupressProcess"; }

  bool init() { return true; }
  bool execute();
  bool finish(){return true;}
};


#endif // brec_prob_map_supress_process_h_
