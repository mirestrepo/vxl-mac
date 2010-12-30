#! /usr/bin/perl -w
use strict;
#
#  This script verifies that each individual VXL header file can be #included.
#  It supplements the many test_include.cxx tests which have #includes for all
#  header files in a particular library: here, inclusion of individual files is
#  tests, but on the other hand here a very long list of include paths is to be
#  used, and moreover this script tries to test all header files, including
#  those which would never be accessed on a certain platform or installation.
#
#  Note: run this script from the VXL root directory.
#  Note: set the vxl build directory appropriately:
my $builddir = $ENV{'HOME'}.'/Linux-i386';

my @include_path = qw(vcl $builddir/vcl core contrib v3p $builddir/v3p/mpeg2/include $builddir/v3p/dcmtk $builddir/core $builddir/core/tests $builddir/core/vnl $builddir/core/vil $builddir/core/vgui $builddir/core/vidl $builddir/contrib/mul/mbl $builddir/contrib/brl/b3p/expat v3p/bzlib v3p/dcmtk v3p/geotiff v3p/jpeg v3p/mpeg2/include v3p/netlib v3p/png v3p/Qv v3p/rply v3p/tiff v3p/zlib contrib/brl contrib/brl/b3p contrib/brl/b3p/expatpp contrib/brl/b3p/shapelib contrib/brl/bbas contrib/brl/bbas/bsta contrib/brl/bmvl contrib/brl/bpro contrib/brl/bpro/bprb contrib/brl/bpro/core contrib/brl/bseg contrib/brl/bseg/bbgm contrib/brl/bseg/bmdl contrib/brl/bseg/boct contrib/brl/bseg/boxm contrib/brl/bseg/boxm/algo contrib/brl/bseg/brec contrib/brl/bseg/bvpl contrib/brl/bseg/bvpl/bvpl_octree contrib/brl/bseg/bvpl/bvpl_octree/pro contrib/brl/bseg/bvxm contrib/brl/bseg/bvxm/pro contrib/brl/expatpp/expat/lib contrib/brl/expatpp/src_pp contrib/gel contrib/gel/mrc contrib/gel/mrc/vpgl contrib/gel/mrc/vpgl/icam contrib/gel/mrc/vpgl/vsph contrib/mul contrib/mul/vpgl contrib/oxl contrib/prip contrib/conversions contrib/oul contrib/rpl contrib/tbl /usr/include/nvidia-current);

my $INC = join ' ',map { "-I$_" } @include_path;

my @FILES;
if (@ARGV) { @FILES = @ARGV; }
else
{
  my $DIRS = join ' ', grep {-d $_} split ' ','core contrib';
  unless ($DIRS) {
    my $V = $ENV{VXLROOT};
    if (defined($V)) { chdir $V; $DIRS = join ' ', grep {-d $_} split ' ','core contrib'; }
  }
  die "Could not find the directories core or contrib, and no command line arguments were given\n" unless ($DIRS);
  @FILES = qx(find $DIRS -type f \\\( -name \\\*.txx -o -name \\\*.h -o -name \\\*.hxx \\\) -print);
  chomp @FILES;
}

foreach my $f (@FILES) {
  next if ($f eq 'core/doc/vxl_doc_rules.h'); # not a source file, only for documentation purposes
  next if ($f =~ m!^core/vgui/(impl|examples)/!); # MFC examples only compile under very specific conditions
  next if ($f =~ m!^contrib/brl/b3p/!); # this is 3rd party software
  next if ($f eq 'core/vnl/tests/test_arithmetic_body.h');   # this file is only to be included from a particular file
  next if ($f eq 'contrib/brl/bpro/bpro_batch/bpro_defs.h'); # idem
  print "======> $f\n";
  my $testfile = $f; $testfile =~ s!.*/!/tmp/!; $testfile =~ s!\..*!.$$.cxx!;
  open F, ">$testfile";
  $f =~ s/^(core|contrib)\///;
  print F "#include <$f>\nint main(){return 0;}\n";
  close F;
  open P, "c++ -c -o /dev/null $INC $testfile 2>&1 |";
  while (<P>) {
    s!^(\S+):(\d+)(:\d+)?: error: !vi +$2 $1\t# !;
    print unless (m!^In file included from ! || m!^ {9,}from !);
  }
  close P;
  unlink $testfile;
}