// This is core/vidl_vil1/vidl_vil1_vob_frame_index.cxx
#include "vidl_vil1_vob_frame_index.h"
//
// this file has been copied from oxl/oxp
// author:  AWF
// copied by l.e.galup
// 10-18-02

#include <vcl_cassert.h>
#include <vcl_cstdio.h>
#include <vcl_fstream.h>
#include <vul/vul_awk.h>

bool vidl_vil1_vob_frame_index::load(vcl_string const& filename)
{
  vcl_vector<vidl_vil1_vob_frame_index_entry> tmp;

  vcl_ifstream f(filename.c_str(), vcl_ios_binary);
  if (!f.good())
  {
    vcl_cerr << "vidl_vil1_vob_frame_index: Cannot read IDX file ["<< filename <<"]\n";
    return false;
  }
  vul_awk awk(f);
  vcl_string tag(awk[0]);
  const int MPEG_IDX = 1;
  const int LBA = 2;
  int idx_type = 0;
  if (tag == "MPEG_IDX") 
    idx_type = MPEG_IDX;
  else if (tag == "LBA")
    idx_type = LBA;
  else
    vcl_cerr << "vidl_vil1_vob_frame_index: WARNING: unknown type [" << awk[0] << "]\n";

  for (int frame=0; awk; ++awk, ++frame)
  {
    // Skip comment and ----- lines
    vidl_vil1_vob_frame_index_entry e;
    if (idx_type == LBA && vcl_sscanf(awk.line(), " %x | %d", &e.lba, &e.frame) == 2)
      tmp.push_back(e);
    int dummy;
    if (idx_type == MPEG_IDX && vcl_sscanf(awk.line(), " %x %x", &e.lba, &dummy) == 2)
    {
      e.frame = frame;
      tmp.push_back(e);
    }
  }
  l = tmp;

  // assert that l is sorted by frame
  for (unsigned int i = 0; i+1 < l.size(); ++i)
    assert(l[i+1].frame > l[i].frame);
  vcl_cerr << "Loaded " << l.size() << " entries from [" << filename << "]\n";
  if (l.size() == 0)
    vcl_cerr << "WARNING: No index entries -- all seeks from start\n";
  return true;
}

int vidl_vil1_vob_frame_index::frame_to_lba_of_prev_I_frame(int f, int* f_actual)
{
  int lo = 0;
  int hi = l.size()-1;
  if (hi < 0 || f < l[lo].frame || f > l[hi].frame) {
    vcl_cerr << "urk: frame " << f << " out of IDX range\n";
    return -1;
  }
  while (lo < hi-1)
  {
    int half = (lo + hi) / 2;
    int f_half = l[half].frame;
    if (f < f_half)
      hi = half;
    else if (f > f_half)
      lo = half;
    else {
      lo = half;
      break;
    }
  }
  // vcl_fprintf(stderr, "vidl_vil1_vob_frame_index: [%5d %5d] -> [%5d %5d]\n", lo, hi, l[lo].frame, l[hi].frame);
  if (f_actual)
    *f_actual = l[lo].frame;
  return l[lo].lba;
}
