//:
// \file
// \author Ian Scott
// \brief test vpdfl_pc_gaussian, building, sampling, saving etc.

// inclusions

#include <vcl_iostream.h>
#include <vcl_string.h>
#include <vcl_ctime.h>
#include <vcl_cmath.h> // for vcl_fabs()

#include <vpdfl/vpdfl_pc_gaussian.h>
#include <vpdfl/vpdfl_pc_gaussian_builder.h>
#include <vpdfl/vpdfl_pc_gaussian_sampler.h>
#include <mbl/mbl_data_array_wrapper.h>
#include <vsl/vsl_binary_loader.h>

//=======================================================================


//: Generate losts of samples using pdf, build new pdf with builder
//  and compare the two
void test_pc_gaussian()
{
  vcl_cout << "**************************" << vcl_endl;
  vcl_cout << " Testing vpdfl_pc_gaussian" << vcl_endl;
  vcl_cout << "**************************" << vcl_endl;

  vsl_add_to_binary_loader(vpdfl_pc_gaussian());
  vsl_add_to_binary_loader(vpdfl_pc_gaussian_builder());

  vpdfl_pc_gaussian_builder builder;

  vpdfl_pc_gaussian pdf;

  int n = 6;
  vnl_vector<double> mean(n),evals(n/2);
  for (int i=0;i<n/2;++i)
  {
    evals(i) = 1+n-(i+1);
  }

  // Initialise to identity
  vnl_matrix<double> evecs(n,n);
  for (int i=0;i<n;++i)
  {
    mean(i) = i;
    for (int j=0;j<n;++j)
      evecs(i,j) = 0.0;
    evecs(i,i) = 1.0;
  }

  vcl_cout<<"Setting evecs:\n"<<evecs<<vcl_endl;
  vcl_cout<<"Setting evals: "<<evals<<vcl_endl;

  pdf.set(mean,evecs,evals,1.5);

  int n_samples = 10000;

  vpdfl_sampler_base* p_sampler = pdf.new_sampler();

    // Generate lots of samples
  vcl_vector<vnl_vector<double> > data(n_samples);
  for (int i=0;i<n_samples;++i)
    p_sampler->sample(data[i]);

  builder.set_proportion_partition(0.75);

  vpdfl_pdf_base* p_pdf_built = builder.new_model();

        // vector<T>::iterator need not be a T* (fsm)
  mbl_data_array_wrapper<vnl_vector<double> > data_array(&data[0]/*.begin()*/, n_samples);

  builder.build(*p_pdf_built, data_array);

  vcl_cout<<"Original PDF: "; vsl_print_summary(vcl_cout, pdf); vcl_cout<<vcl_endl;
  vcl_cout<<"Rebuilt PDF: "; vsl_print_summary(vcl_cout, p_pdf_built); vcl_cout<<vcl_endl;
  vcl_cout<<vcl_endl;
  vcl_cout<<"PDF sampler: "; vsl_print_summary(vcl_cout, p_sampler); vcl_cout<<vcl_endl;

// Test the IO ================================================
  vpdfl_builder_base* p_builder = & builder;
  vpdfl_pdf_base* p_pdf = & pdf;

  TEST("mean of built model",vnl_vector_ssd(pdf.mean(), p_pdf_built->mean())<0.1,true);

  vcl_cout<<vcl_endl<<"=================Testing Fast log_p():"<<vcl_endl;

  vnl_vector<double> x;
  p_sampler->sample(x);
  vcl_cout << "Fast log_p() = "<<pdf.log_p(x)<<"\tSlow log_p() = "<<pdf.vpdfl_gaussian::log_p(x)<<vcl_endl;

#ifdef TEST_RELATIVE_SPEEDS_OF_FASTER_LOGP
  double v;
  long t0,t1;
  t0 = vcl_clock();
  for (int i =0; i<10000; i++)
    v = pdf.log_p(x);
  t1 = vcl_clock();
  vcl_cout <<"Time for fast log_p(): " << (t1-t0)/(10.0*double(CLOCKS_PER_SEC)) << "ms" <<vcl_endl;
  t0 = vcl_clock();
  for (int i =0; i<10000; i++)
    v = pdf.vpdfl_gaussian::log_p(x);
  t1 = vcl_clock();
  vcl_cout <<"Time for slow log_p(): " << (t1-t0)/(10.0*double(CLOCKS_PER_SEC)) << "ms" <<vcl_endl;
#endif

  vcl_cout<<vcl_endl<<"=================Testing I/O:"<<vcl_endl;
  vsl_b_ofstream bfs_out("test_pc_gaussian.bvl.tmp");
  TEST ("Created test_pc_gaussian.bvl.tmp for writing",
             (!bfs_out), false);

  vsl_b_write(bfs_out,pdf);
  vsl_b_write(bfs_out,builder);
  vsl_b_write(bfs_out,p_pdf);
  vsl_b_write(bfs_out,p_builder);
  bfs_out.close();

  vpdfl_pc_gaussian          pdf_in;
  vpdfl_pc_gaussian_builder  builder_in;
  vpdfl_pdf_base*            p_base_pdf_in = NULL;
  vpdfl_builder_base*        p_base_builder_in = NULL;


  vsl_b_ifstream bfs_in("test_pc_gaussian.bvl.tmp");
  TEST ("Opened test_pc_gaussian.bvl.tmp for reading", (!bfs_in), false);

  vsl_b_read(bfs_in, pdf_in);
  vsl_b_read(bfs_in, builder_in);
  vsl_b_read(bfs_in, p_base_pdf_in);
  vsl_b_read(bfs_in, p_base_builder_in);
  bfs_in.close();

  vcl_cout<<"Original PDF: "; vsl_print_summary(vcl_cout, pdf); vcl_cout<<vcl_endl;
  vcl_cout<<"Original builder: "; vsl_print_summary(vcl_cout, builder); vcl_cout<<vcl_endl;
  vcl_cout<<vcl_endl;

  vcl_cout<<"Loaded PDF: "; vsl_print_summary(vcl_cout, pdf_in); vcl_cout<<vcl_endl;
  vcl_cout<<"Loaded builder: "; vsl_print_summary(vcl_cout, builder_in); vcl_cout<<vcl_endl;
  vcl_cout<<vcl_endl;


  TEST("Original Model == Loaded model",
    pdf.mean()==pdf_in.mean() &&
    pdf.variance()==pdf_in.variance() &&
    pdf.eigenvals() == pdf_in.eigenvals() &&
    pdf.eigenvecs() == pdf_in.eigenvecs() &&
    pdf.log_k() == pdf_in.log_k() &&
    pdf.log_k_principal() == pdf_in.log_k_principal() &&
    pdf.n_principal_components() == pdf_in.n_principal_components(),
    true);
  TEST("Original Model == model loaded by base ptr",
    pdf.mean()==p_base_pdf_in->mean() &&
    pdf.variance()==p_base_pdf_in->variance() &&
    pdf.is_a()==p_base_pdf_in->is_a(),
    true);
  TEST("Original Builder == Loaded builder",
    builder.min_var()==builder_in.min_var() &&
    builder.fixed_partition()==builder_in.fixed_partition(),
    true);
  TEST("Original Builder == Builder loaded by base ptr",
    builder.min_var()==p_base_builder_in->min_var() &&
    builder.is_a()==p_base_builder_in->is_a(),
    true);


  vcl_cout << "========Testing PDF Thresholds==========";
  vpdfl_sampler_base *p_sampler2 = p_pdf_built->new_sampler();
  unsigned pass=0, fail=0;
  double thresh = p_pdf_built->log_prob_thresh(0.9);
  vcl_cout << vcl_endl << "log density threshold for passing 90%: " << thresh << vcl_endl;
  for (unsigned i=0; i < 1000; i++)
  {
    p_sampler2->sample(x);
    if (p_pdf_built->log_p(x) > thresh)
      pass ++;
    else
      fail ++;
  }
  vcl_cout << "In a sample of 1000 vectors " << pass << " passed and " << fail <<  " failed." << vcl_endl;
  TEST("880 < pass < 920", pass > 880 && pass < 920, true);
  pass=0; fail=0;
  thresh = p_pdf_built->log_prob_thresh(0.1);
  vcl_cout << vcl_endl << vcl_endl << "log density threshold for passing 10%: " << thresh << vcl_endl;
  for (unsigned i=0; i < 1000; i++)
  {
    p_sampler2->sample(x);
    if (p_pdf_built->log_p(x) > thresh)
      pass ++;
    else
      fail ++;
  }
  vcl_cout << "In a sample of 1000 vectors " << pass << " passed and " << fail <<  " failed." << vcl_endl;
  TEST("70 < pass < 130", pass > 70 && pass < 130, true);
  delete p_sampler2;


  delete p_pdf_built;
  delete p_sampler;
  delete p_base_pdf_in;
  delete p_base_builder_in;
}


TESTMAIN(test_pc_gaussian);
