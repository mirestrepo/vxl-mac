#include <vcl_cassert.h>
#include <vcl_cstring.h>
#include <vcl_fstream.h>

#include <vul/vul_awk.h>
#include <vul/vul_file.h>

#include <vxl_config.h>

#include <oxp/oxp_bunch_of_files.h>

oxp_bunch_of_files::oxp_bunch_of_files(char const* fmt)
{
  open(fmt);
}

oxp_bunch_of_files::oxp_bunch_of_files()
{
}

bool oxp_bunch_of_files::open(char const* fmt)
{
  current_file_index = -1;

  // fmt could be a "lst" file, or a single vob or mpg or a %d list
  int l = vcl_strlen(fmt);
  if (l > 4 && vcl_strcmp(fmt + l - 4, ".lst") == 0) {
    vcl_ifstream f(fmt);
    assert(f.good());
    for (vul_awk awk(f); awk; ++awk) {
      if (awk.NF() > 0) {
        vcl_cerr << awk[0] << vcl_endl;
        filenames.push_back(awk[0]);
      }
    }
  }
  else if (vcl_strchr(fmt, '%') ) {
    // Assume a list.  Could start from some low number... Could glob.  hmmm.
    bool found_one = false;
    for (int i = 0; ; ++i) {
      char buf[1024];
      vcl_sprintf(buf, fmt, i);
      int s = vul_file::size(buf);
      if (s > 0) {
        found_one = true;
        filenames.push_back(buf);
        filesizes.push_back(s);
      }
      else {
        // If got at least one so far, then bail now
        if (found_one)
          break;
        else {
          // If not found one yet, and still only tried < 10, then
          // keep trying.
          if (i > 10)
            break;
        }
      }
    }
  }
  else {
    // No %, not .lst: assume it's an mpeg/vob
    filenames.push_back(fmt);
  }

  unsigned int n = filenames.size();

  if (n == 0) {
    vcl_fprintf(stderr, "ERROR: Could not turn [%s] into a list of files\n", fmt);
    return false;
  }

  // Fill in sizes if not done already
  if (filesizes.size() < n) {
    filesizes.resize(n);
    for (unsigned int i = 0; i < n; ++i) {
      int s = vul_file::size(filenames[i].c_str());
      if (s <= 0) {
        vcl_fprintf(stderr, "WARNING: Zero size file [%s]\n", filenames[i].c_str());
      }
      filesizes[i] = s;
    }
  }

  // Set up file ptr etc.
  current_file_index = 0;

  // Fill start_bytes
  start_byte.resize(n);
  start_byte[0] = 0;
  for (unsigned int i = 1; i < filenames.size(); ++i)
    start_byte[i] = (vxl_int_64)start_byte[i-1] + (vxl_int_64)filesizes[i-1];

  // Open them all
  fps.resize(n);
  for (unsigned int i = 0; i < filenames.size(); ++i) {
    char const* fn = filenames[i].c_str();
    fps[i] = vcl_fopen(fn, "rb");
    if (!fps[i]) {
      vcl_fprintf(stderr, "ERROR: Could not open [%s]\n", fn);
      current_file_index = -1;
      return false;
    }
  }

  // Summarize:
  vcl_fprintf(stderr, "files: sizeof offset_t = %d\n", (int)sizeof(offset_t));
  for (unsigned int i = 0; i < n; ++i)
    vcl_fprintf(stderr, "   %s  %ld\n", filenames[i].c_str(), (long)start_byte[i]);
  vcl_fprintf(stderr, "\n");

  return true;
}

void oxp_bunch_of_files::seek(offset_t to)
{
  int newindex = -1;
  for (unsigned int i = 1; i < filesizes.size(); ++i)
    if (start_byte[i] > to) {
      newindex = i-1;
      break;
    }

  if (newindex == -1) {
    int i = filesizes.size() - 1;
    // Know start_byte[i] <= to
    if (to < start_byte[i] + (offset_t)filesizes[i])
      newindex = i;
  }

  if (newindex == -1) {
    vcl_fprintf(stderr, "ERROR: Could not seek to [%lu]\n", (unsigned long)to);
    return;
  }

  current_file_index = newindex;

  offset_t file_ptr = to - start_byte[current_file_index];
  vcl_fprintf(stderr, " si = %20g to = %20g\n", (double)start_byte[current_file_index], (double) to);
  assert(file_ptr >= 0);
  assert(file_ptr < (offset_t)filesizes[current_file_index]);

  vcl_fseek(fps[current_file_index], file_ptr, SEEK_SET);
}

int oxp_bunch_of_files::tell() const
{
  return start_byte[current_file_index] + ftell(fps[current_file_index]);
}

int oxp_bunch_of_files::read(void* buf, unsigned int len)
{
  unsigned long space_left_in_this_file = filesizes[current_file_index] - ftell(fps[current_file_index]);

  int bytes_from_curr = len;
  int bytes_from_next = 0;
  if (space_left_in_this_file < len) {
    bytes_from_curr = space_left_in_this_file;
    bytes_from_next = len - space_left_in_this_file;
  }

  if (bytes_from_next == 0)
    return vcl_fread(buf, 1, len, fps[current_file_index]);

  int n1 = vcl_fread(buf, 1, bytes_from_curr, fps[current_file_index]);
  if (n1 < bytes_from_curr)
    // First read stopped short, don't even bother with next one
    return n1;
  if ((unsigned int)(current_file_index+1) == fps.size())
    // First was OK, and we've run out of files
    return n1;

  // First read was OK.  Advance to next file.
  ++current_file_index;
  int n2 = vcl_fread((unsigned char*)buf + n1, 1, bytes_from_next, fps[current_file_index]);
  return n1 + n2;
}

oxp_bunch_of_files::~oxp_bunch_of_files()
{
  for (unsigned int i = 0; i < fps.size(); ++i)
    vcl_fclose(fps[i]);
}

