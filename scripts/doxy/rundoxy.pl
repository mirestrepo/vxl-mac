#!/bin/sh
# -*- perl -*-
exec perl -w -x $0 ${1+"$@"}
#!perl
#line 6
# If Windows barfs at line 3 here, you will need to run perl -x vxl_doxy.pl
# You can set up as a permanent file association using the following commands
#  >assoc .pl=PerlScript
#  >ftype PerlScript=C:\Perl\bin\Perl.exe -x "%1" %*

use Cwd;
use Getopt::Std;

#-----------------------------------------------------------
# rundoxy.pl -v vxlsrc -s script_dir -l library_list_file
#            -n library_name [-o outputdir]
# Creates documentation for given library in
# vxl_src/Doxy/library_name/html
#-----------------------------------------------------------

#-----------------------------------------------------------

sub xec
{
  my $command = $_[0];
  print "exec [$command]\n";
  my $op = `$command`;
  print $op;
  if ( $? != 0 )
  {
    print "<$command> failed\n";
  }
}

#-----------------------------------------------------------
#  ($prefix,@deps) = get_dependencies( $library, $library_list_file );
#  Read through library list file, building up list of libraries
#  which precede $library - it is assumed to depend only on these.

sub get_dependencies
{
  my ($library,$file)  = @_;
  my $package;
  $prefix = "";
  @stuff = ();
  @liblist = ();

  # search the file for an entry

  open(IN, $file) || die "can't open $file\n";
  while (<IN>)
  {
    # ignore empty lines
    if ( /^\s*$/ ) { next; }

    # ignore comments
    if ( /^#/ ) { next; }

    # ignore lines containing "package:"
    if ( /package:/ ) { next; }

    chomp;
    ($package, $libry, $pref, @stuff) = split /\s/;

    $packlib = $package . "/" . $libry;

    # Check for special case (eg . vcl)
    if ($package eq ".") { $packlib = $library; }

    if ( $packlib eq $library )
    {
      $prefix = $pref;
      last;
    }
		# Add library to list of those seen
    @liblist = (@liblist,$packlib);
  }
  close(IN);
  return ($prefix, @liblist);
}

#-----------------------------------------------------------
#  create_doxyfile($dfXXX, $doxydir, $opfile, $library, $pref, $deps, $strip, $header, $infilter,$havedot);

sub  create_doxyfile
{
  my ($dfXXX, $doxydir, $opfile, $library, $pref, $deps, $strip, $header, $infilter,$havedot) = @_;
  open(IN, "$dfXXX")  || die "can't open file $dfXXX\n";
  open(OUT, ">$opfile") || die "can't open file $opfile\n";

  $libname = $library;
  $libname =~ s/\//_/g;

  $tagfile = $doxydir . "/tags/" . $libname . ".tag";
  $outputdir = $doxydir . "/" . $library ;
  $abspath   = $outputdir . "/html";

  while (<IN>)
  {
    # change the library name, the prefixes to strip

    s/XXX/$library/g;
    s/SSSPREF/$pref/g;
    s/SSSPATH/$strip/g;
    s/SSSHEADER/$header/g;
    s/SSSINFILTER/$infilter/g;
    s/SSSOUTPUTDIR/$outputdir/g;
    s/SSSTAGFILE/$tagfile/g;
    s/SSSABSPATH/$abspath/g;
    s/SSSHAVEDOT/$havedot/g;

    # add the tagfile dependencies

    if ( /^TAGFILES/ )
    {
      chomp;
      print OUT;
      $size = @deps;
      if ( $size != 0 )
      {
        foreach $dname (@deps)
        {
        $depname =$dname;
        $depname =~ s/\//_/g;
        print OUT " \\\n";
        print OUT "\t\t$doxydir/tags/$depname.tag";
        }
      }
      print OUT "\n";
      next;
    }

    print OUT;
  }

  close(IN);
  close(OUT);
}

# "YES"/"NO" = check_havedot($doxyoutputdir)
sub check_havedot
{
  my ($doxyoutputdir) = @_;
  `dot --version > $doxyoutputdir/checkdot.out  2>&1`;
  if ($? != 0)
  {
    return "NO";
  }

  return "YES";
}

#-----------------------------------------------------------
# Main
#-----------------------------------------------------------

my %options;
getopts('v:s:l:n:o:', \%options);

my $vxlsrc = $options{v} || "";
my $script_dir = $options{s} || "$vxlsrc/scripts/doxy";
my $library_list_file = $options{l} || "$script_dir/data/library_list.txt";
my $library = $options{n} || "";
my $doxydir = $options{o} || "$vxlsrc/Doxy";

if ( ! $vxlsrc || !$library_list_file || !$library)
{
  print "syntax is:\n";
  print "rundoxy.pl -v vxlsrc -s script_dir ";
    print " -l library_list_file -n library_name [-o outputdir]\n\n";
  print "Creates documentation for given library in\n";
  print "vxl_src/Doxy/library_name/html\n";
  exit(1);
}
#print $library;

chdir $vxlsrc || die "Unable to chdir to $vxlsrc\n";

if (! -e $library)
{
  print "No subdirectory called $library exists here\n";
  $here = cwd();
  print "Here is currently $here\n";
  exit(1);
}

if (  ! -e $library_list_file )
{
  print "<$library_list_file> does not exist\n";
  exit(1);
}

# Check that a directory called $doxydir exists
if (! -e $doxydir)
{
  print "Creating $doxydir\n";
  mkdir $doxydir,0777 || die "Can't create directory $doxydir\n";
}

$doxyoutputdir = $doxydir . "/output";

# Check that a subdirectory called Doxy/output exists
if (! -e $doxyoutputdir)
{
  print "Creating $doxyoutputdir\n";
  mkdir $doxyoutputdir,0777 || die "Can't create directory $doxyoutputdir\n";
}

$tagsdir = $doxydir . "/tags";

# Check that a subdirectory called Doxy/tags exists
if (! -e $tagsdir)
{
  print "Creating $tagsdir\n";
  mkdir $tagsdir,0777 || die "Can't create directory $tagsdir\n";
}

# Create library name  (vnl -> vnl,  vnl/algo -> vnl_algo etc)
$libname = $library;
$libname =~ s/\//_/g;

#get the dependencies and the preficx to strip

($pref,@deps) = get_dependencies($library,$library_list_file);
#print "$pref ";
#print "(@deps)\n";

# create the file Doxyfile.<library> from the skeleton file
# use the deps and the prefix-to-strip.
# $strip is the cwd() to be strippe from include paths in the doc.

$dfXXX = "$script_dir/Doxyfile.XXX";
$opfile = "$doxydir/output/Doxyfile.$libname";
$strip = cwd(); chomp $strip;
$strip = $strip . "/";
$header = "$script_dir/doxy_header.html";
$infilter = $vxlsrc . "/vxl/doc/vxl_doxy.pl";
$havedot = check_havedot($doxyoutputdir);

#print "$strip\n";
create_doxyfile($dfXXX, $doxydir, $opfile, $library, $pref, $deps, $strip, $header, $infilter, $havedot);


# Work out how to get from current library to base
# "dir1" -> "..",   "dir1/dir2" -> "../.." etc
$relpath = $library;
$relpath =~ s/[^\/]+/\.\./g;

# create the command to install the docs

$command = "./installdox.pl ";
foreach $dep (@deps)
{
  # Deduce tag file name  (vnl -> vnl.tag,  vnl/algo -> vnl_algo.tag etc)
  $deptag = $dep . ".tag";
  $deptag =~ s/\//_/g;

  $command = $command  . "-l $deptag\@";
  $command = $command  . "$relpath/../$dep/html ";
}
#print "$command\n";


$doxyoutfile = $doxyoutputdir . "/" . $libname . ".doxy_out";
$doxyoutfile2 = $doxyoutputdir . "/" . $libname . "_install.doxy_out";

# now execute everything

$here = cwd();
xec("doxygen $opfile > $doxyoutfile 2>&1 ");
chdir "$doxydir/$library/html" || die "can't cd to $doxydir/$library/html";

if (! -e "installdox")
{
  print "No installdox available for $library.  doxygen failed to create one.\n";
}
else
{
  # Ensure installdox gets treated as a perl script by Windows
  rename "installdox" ,"installdox.pl";
  xec("$command > $doxyoutfile2 2>&1 ");
}

chdir $here;

#-----------------------------------------------------------
