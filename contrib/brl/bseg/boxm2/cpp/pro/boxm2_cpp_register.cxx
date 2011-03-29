#include "boxm2_cpp_register.h"

#include <bprb/bprb_macros.h>
#include <bprb/bprb_batch_process_manager.h>
#include <bprb/bprb_func_process.h>

#include "boxm2_cpp_processes.h"

void boxm2_cpp_register::register_datatype()
{
}

void boxm2_cpp_register::register_process()
{
  // utilities
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_cpp_render_expected_image_process, "boxm2CppRenderExpectedImageProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_cpp_update_image_process, "boxm2CppUpdateImageProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_cpp_refine_process2, "boxm2CppRefineProcess");
  REG_PROCESS_FUNC_CONS(bprb_func_process, bprb_batch_process_manager, boxm2_cpp_change_detection_process2, "boxm2CppChangeDetectionProcess");
}