#ifndef bomx2_opencl_processor_h
#define bomx2_opencl_processor_h

#include <boxm2/boxm2_processor.h>
#include <brdb/brdb_value_sptr.h>
#include <vcl_vector.h>

class boxm2_opencl_processor: public boxm2_processor
{
 public:
    boxm2_opencl_processor(){}
    ~boxm2_opencl_processor(){}

    virtual bool  init();
    virtual bool  run(boxm2_process * process, vcl_vector<brdb_value_sptr> & input, vcl_vector<brdb_value_sptr> & output);
    virtual bool  finish();
};

#endif
