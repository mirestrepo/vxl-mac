// Copyright: (C) 2000 British Telecommunications PLC

//:
// \file
// \brief Test clsfy_random_classifier, clsfy_k_nearest_neighbour and clsfy_rbf_parzen_window
// \author Ian Scott
// Test construction, IO etc.

#include <vcl_iostream.h>
#include <vcl_iomanip.h>
#include <vcl_ios.h>
#include <vcl_limits.h>
#include <vcl_vector.h>
#include <vcl_algorithm.h>
#include <vcl_sstream.h>

#include <vcl_string.h>
#include <clsfy/clsfy_knn_builder.h>
#include <clsfy/clsfy_k_nearest_neighbour.h>
#include <clsfy/clsfy_rbf_parzen.h>
#include <clsfy/clsfy_random_builder.h>
#include <clsfy/clsfy_random_classifier.h>
#include <vsl/vsl_binary_loader.h>
#include <vnl/vnl_math.h>
#include <mbl/mbl_mz_random.h>
#include <mbl/mbl_data_array_wrapper.h>
#include <vpdfl/vpdfl_axis_gaussian_sampler.h>
#include <vpdfl/vpdfl_axis_gaussian.h>

#include <vnl/vnl_test.h>

//: Test clsfy_k_nearest_neighbour and clsfy.rbf_parzen_window
void test_k_nearest_neighbour()
{
  vcl_cout << "\n\n\n"
           << "**************************************************************" << vcl_endl
           << " Testing clsfy_k_nearest_neighbour and clsfy.rbf_parzen_window" << vcl_endl
           << "**************************************************************" << vcl_endl;

  vcl_cout<<"\n======== TESTING CONSTRUCTION ==========="<<vcl_endl;


  vcl_vector<vpdfl_axis_gaussian_sampler *> generator(4);//
  const unsigned nDims = 2;
  vnl_vector<double> mean0(nDims), var0(nDims), mean1(nDims), var1(nDims), mean2(nDims), var2(nDims), mean3(nDims), var3(nDims);
  vpdfl_axis_gaussian PDF0, PDF1, PDF2, PDF3;


  mean0.fill(0.0); mean0(0) = 1.5;
  mean1.fill(0.0); mean1(0) = -1.5;
  mean2.fill(0.0); mean2(1) = 1.5;
  mean3.fill(0.0); mean3(1) = -1.5;

  var0.fill(0.1); var0(0) = 0.2;
  var1.fill(0.1); var1(0) = 0.2;
  var2.fill(0.1); var2(1) = 0.2;
  var3.fill(0.1); var3(1) = 0.2;

  PDF0.set(mean0, var0);
  PDF1.set(mean1, var1);
  PDF2.set(mean2, var2);
  PDF3.set(mean3, var3);

  generator[0] = (vpdfl_axis_gaussian_sampler *)PDF0.new_sampler();
  generator[1] = (vpdfl_axis_gaussian_sampler *)PDF1.new_sampler();
  generator[2] = (vpdfl_axis_gaussian_sampler *)PDF2.new_sampler();
  generator[3] = (vpdfl_axis_gaussian_sampler *)PDF3.new_sampler();
  mbl_mz_random rng;
  rng.reseed(333233);

  const unsigned nSamples = 200;
  const unsigned nTestSamples = 500;

  vcl_vector<unsigned> labels(nSamples);
  vcl_vector<unsigned> testLabels(nTestSamples);
  vcl_vector<vnl_vector<double> > data(nSamples);
  vcl_vector<vnl_vector<double> > testData(nTestSamples);
  vnl_vector<double> s;
  vcl_vector<unsigned> labelcount(4);
  vcl_fill(labelcount.begin(), labelcount.end(), 0u);

  for (int i=0; i<nSamples; i++)
  {
    int c = rng.lrand32(0, 3);
    labels[i] = c/2;
    labelcount[c] ++;
    generator[c]->sample(s);
    data[i] = s;
  }
  vcl_cout << "The number of labels from each generators are respectively "
           << labelcount[0] << ' ' << labelcount[1] << ' ' << labelcount[2]
           << ' ' << labelcount[3] <<  vcl_endl;

  for (int i=0; i<nTestSamples; i++)
  {
    int c = rng.lrand32(0, 3);
    testLabels[i] = c/2;
    generator[c]->sample(s);
    testData[i] = s;
  }
  delete generator[0];
  delete generator[1];
  delete generator[2];
  delete generator[3];

  mbl_data_array_wrapper<vnl_vector<double> > training_set_inputs(data);
  mbl_data_array_wrapper<vnl_vector<double> > test_set_inputs(testData);

  vcl_cout << "****************Testing over descision space****************" <<vcl_endl;


  vnl_vector<double> x(nDims);
  vcl_vector<double> out(1);
  x.fill(0.0);
  vcl_cout << "x(2) varies across from -2 to + 2" << vcl_endl
           << "x(1) varies down from -2 to + 2" << vcl_endl;

  clsfy_k_nearest_neighbour knn;
  knn.set(data, labels);
  knn.set_k(3);
  vcl_cout << vcl_endl << "KNN output"<<vcl_endl
           << vcl_setprecision(4);
  for (x(0) = -2; x(0) <= 2 ; x(0) += 0.25)
  {
    for (x(1) = -2; x(1) <= 2 ; x(1) += 0.25)
    {
      knn.class_probabilities(out, x);
      vcl_cout << vcl_fixed << vcl_setw(3) << out[0] << ' ';
    }
    vcl_cout << vcl_endl;
  }

  for (x(0) = -2; x(0) <= 2 ; x(0) += 0.25)
  {
    for (x(1) = -2; x(1) <= 2 ; x(1) += 0.25)
    {
      vcl_cout << knn.classify(x);
    }
    vcl_cout << vcl_endl;
  }

  clsfy_rbf_parzen win;
  win.set(data, labels);
  win.set_rbf_width(0.141);
  win.set_power(10);
  vcl_cout << vcl_endl << "Training data distribution"<<vcl_endl;

  vcl_cout << vcl_setprecision(1);
  for (x(0) = -2; x(0) <= 2 ; x(0) += 0.25)
  {
    for (x(1) = -2; x(1) <= 2 ; x(1) += 0.25)
    {
      vcl_cout << vcl_fixed << vcl_setw(4) << win.weightings(x) << ' ';
    }
    vcl_cout << vcl_endl;
  }

  vcl_cout << vcl_endl << "RBF Window Classifier output"<<vcl_endl;

  vcl_cout << vcl_setprecision(4);
  win.set_rbf_width(0.3);
  win.set_power(2);
  for (x(0) = -2; x(0) <= 2 ; x(0) += 0.25)
  {
    for (x(1) = -2; x(1) <= 2 ; x(1) += 0.25)
    {
      win.class_probabilities(out, x);
      vcl_cout << vcl_fixed << vcl_setw(3) << out[0] << ' ';
    }
    vcl_cout << vcl_endl;
  }

  for (x(0) = -2; x(0) <= 2 ; x(0) += 0.25)
  {
    for (x(1) = -2; x(1) <= 2 ; x(1) += 0.25)
    {
      vcl_cout << win.classify(x);
    }
    vcl_cout << vcl_endl;
  }


  clsfy_random_builder rb;
  rb.set_confidence(0.0);
  clsfy_random_classifier rc;
  rc.reseed(456456ul);
  rb.build(rc, training_set_inputs, 1, labels);

  vcl_cout  << vcl_endl << "Random Classifier output"<<vcl_endl;

  vcl_cout << vcl_setprecision(4);
  for (x(0) = -2; x(0) <= 2 ; x(0) += 0.25)
  {
    for (x(1) = -2; x(1) <= 2 ; x(1) += 0.25)
    {
      rc.class_probabilities(out, x);
      vcl_cout << vcl_fixed << vcl_setw(3) << out[0] << ' ';
    }
    vcl_cout << vcl_endl;
  }

  for (x(0) = -2; x(0) <= 2 ; x(0) += 0.25)
  {
    for (x(1) = -2; x(1) <= 2 ; x(1) += 0.25)
    {
      vcl_cout << rc.classify(x);
    }
    vcl_cout << vcl_endl;
  }


  vcl_cout << "****************Testing over testing set**************" <<vcl_endl;
#if 0
  vcl_cout << clsfy_test_error(knn, test_set_inputs, testLabels) << vcl_endl;
#endif

  TEST("Test error on clsfy_k_nearest_neighbour close to 0.0",
       clsfy_test_error(knn, test_set_inputs, testLabels) < 0.02, true);
#if 0
  vcl_cout << clsfy_test_error(win, test_set_inputs, testLabels) << vcl_endl;
#endif

  TEST("test error on clsfy_rbf_parzen_window close to 0.0",
       clsfy_test_error(win, test_set_inputs, testLabels) < 0.02, true);

  vcl_vector<double> probs(2);
  probs[0] = 0.25;
  probs[1] = 0.75;
  rc.set_probs(probs);

#if 0
  vcl_cout << clsfy_test_error(rc, test_set_inputs, testLabels) << vcl_endl;
#endif
  TEST("test error on clsfy_random_classifier close to 0.5",
       vnl_math_abs(clsfy_test_error(rc, test_set_inputs, testLabels) - 0.5) < 0.05, true);

  vcl_cout << "****************Testing builder**************" <<vcl_endl;


  rc.class_probabilities(out, x);
  TEST("Random classifier indicates correct input size", rc.n_dims(), 2);
  TEST("Random classifier indicates correct output size", rc.n_classes(), 1);
  TEST("Random classifier gives correct output size", out.size(), 1);


  vcl_cout << "****************Testing classifier IO**************" <<vcl_endl;

  vcl_string test_path = "test_k_nearest_neighbour.bvl.tmp";


  vsl_add_to_binary_loader(clsfy_k_nearest_neighbour());
  vsl_add_to_binary_loader(clsfy_rbf_parzen());
  vsl_b_ofstream bfs_out(test_path);
  TEST(("Opened " + test_path + " for writing").c_str(), (!bfs_out ), false);

  vsl_b_write(bfs_out,knn);
  vsl_b_write(bfs_out,win);
  vsl_b_write(bfs_out,(clsfy_classifier_base *)&knn);
  vsl_b_write(bfs_out,(clsfy_classifier_base *)&win);
  bfs_out.close();


  clsfy_k_nearest_neighbour knn_in;
  clsfy_rbf_parzen win_in;
  clsfy_classifier_base *p_base_class_knn_in=0, *p_base_class_win_in=0;

  vsl_b_ifstream bfs_in(test_path);
  TEST(("Opened " + test_path + " for writing").c_str(), (!bfs_out ), false);

  vsl_b_read(bfs_in, knn_in);
  vsl_b_read(bfs_in, win_in);
  vsl_b_read(bfs_in, p_base_class_knn_in);
  vsl_b_read(bfs_in, p_base_class_win_in);

  bfs_in.close();


  vcl_cout<<"Saved KNN: " << knn << vcl_endl
          <<"Loaded KNN: " << knn_in << vcl_endl
          <<"Loaded KNN: " << p_base_class_knn_in << vcl_endl

          <<"Saved Win: " << win << vcl_endl
          <<"Loaded Win: " << win_in << vcl_endl
          <<"Loaded Win: " << p_base_class_win_in << vcl_endl;


  const vnl_vector<double> probe(2,2.0);

  TEST("Original KNN == Loaded KNN",
       knn.n_classes() == knn_in.n_classes() &&
       knn.n_dims() == knn_in.n_dims() &&
       knn.training_classes() == knn_in.training_classes() &&
       knn.training_vectors() == knn_in.training_vectors() &&
       knn.k() == knn_in.k(),
       true);


  vcl_cout << "knn.log_l(2.0, 2.0) = " << knn.log_l(probe) << vcl_endl;
  TEST("Original KNN(2.0, 2.0) == Loaded KNN(2.0, 2.0)",
       knn.log_l(probe) == knn_in.log_l(probe),
       true);

  TEST("Original KNN == KNN loaded by base ptr",
       knn.n_classes() == p_base_class_knn_in->n_classes() &&
       knn.n_dims() == p_base_class_knn_in->n_dims() &&
       knn.is_a() == p_base_class_knn_in->is_a(),
       true);

  TEST("Original KNN(2.0, 2.0) == Loaded by base ptr KNN(2.0, 2.0)",
       knn.log_l(probe) == p_base_class_knn_in->log_l(probe),
       true);

  TEST("Original Parzen == Loaded Parzen",
       win.n_classes() == win_in.n_classes() &&
       win.n_dims() == win_in.n_dims() &&
       win.training_classes() == win_in.training_classes() &&
       win.training_vectors() == win_in.training_vectors() &&
       win.rbf_width() == win_in.rbf_width(),
       true);

  vcl_cout << "win.log_l(2.0, 2.0) = " << win.log_l(probe) << vcl_endl;
  TEST("Original Parzen(2.0, 2.0) == Loaded Parzen(2.0, 2.0)",
       win.log_l(probe), win_in.log_l(probe));

  TEST("Original Parzen == Parzen loaded by base ptr",
       win.n_classes() == p_base_class_win_in->n_classes() &&
       win.n_dims() == p_base_class_win_in->n_dims() &&
       win.is_a() == p_base_class_win_in->is_a(),
       true);

  TEST("Original Parzen(2.0, 2.0) == Loaded by base ptr Parzen(2.0, 2.0)",
       win.log_l(probe), p_base_class_win_in->log_l(probe));

  vcl_cout << vcl_setprecision(6) << vcl_resetiosflags(vcl_ios_floatfield);

  delete p_base_class_knn_in;
  delete p_base_class_win_in;
  vsl_delete_all_loaders();
}

TESTMAIN(test_k_nearest_neighbour);
