//:
// \file
#include <testlib/testlib_test.h>
#include <brad/brad_phongs_model_est.h>
#include <vcl_cmath.h>
#include <vnl/vnl_double_3.h>
#include <vnl/vnl_random.h>
#include <vnl/vnl_math.h>
#include <vnl/vnl_identity_3x3.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>
#include <vcl_iostream.h>
#include <vcl_vector.h>

//: Test the sun_pos class
static void test_phongs_model_est()
{
    // first define model parameters
    double kd=0.5;
    double ks=0.5;

    double gamma=6;

    vnl_double_3 normal(vcl_sin(0.75)*vcl_cos(0.0),vcl_sin(0.75)*vcl_sin(0.0),vcl_cos(0.75));

    double sun_elev = 0.325398;
    double sun_phi = -3.69;
    vnl_double_3 lv(vcl_sin(sun_elev)*vcl_cos(sun_phi),
                    vcl_sin(sun_elev)*vcl_sin(sun_phi),
                    vcl_cos(sun_elev));
    vnl_identity_3x3 I;
    vnl_double_3 rlv=(I-outer_product<double>(normal,normal)-outer_product<double>(normal,normal))*lv;

    // vary the viewpoint direction to obtain the samples
    unsigned int num_samples=30;
    vnl_vector<double> samples(num_samples);
    vnl_vector<double> samples_weights(num_samples);
    vnl_vector<double> camera_elev(num_samples);
    vnl_vector<double> camera_azim(num_samples);
    vnl_random rand;
    for (unsigned i=0; i < num_samples; ++i)
    {
        double elev = rand.drand32(vnl_math::pi/6,vnl_math::pi/3);
        double azim = rand.drand32(2*vnl_math::pi);
        vnl_double_3 vv(vcl_sin(elev)*vcl_cos(azim),
                        vcl_sin(elev)*vcl_sin(azim),
                        vcl_cos(elev));
        double obs = kd * dot_product(lv,normal) + ks* vcl_pow(dot_product<double>(vv,rlv),gamma);
        samples[i]=obs;
        camera_elev[i]=elev;
        camera_azim[i]=azim;

        samples_weights[i]=1.0;
        samples[i]=obs;
    }
#if 0
    //add noise
    for (unsigned i = 0;i < num_samples ;i++)
       samples[i]+=rand.drand32(-0.1,0.1);
#endif
    // estimate the model from the samples.
    brad_phongs_model_est f(sun_elev,sun_phi,camera_elev,camera_azim,samples,samples_weights,true);
    vnl_vector<double> x(5);
    x[0]=0.1;
    x[1]=0.1;
    x[2]=3.0;
    x[3]=0.2;
    x[4]=0.5;
    vnl_levenberg_marquardt lm(f);
    lm.set_verbose(true);
    double min_error=1e5;
    vnl_vector<double> argminx;
    for (float theta=0;theta<vnl_math::pi/2;theta+=0.1)
    {
        for (float k1=0.0;k1<1;k1+=0.1)
        {
            for (float k2=0.0;k2<1;k2+=0.1)
            {
                x[2]=6.0;
                x[4]=0.5;

                x[0]=k1;
                x[1]=k2;
                x[3]=theta;
                lm.minimize(x);
                if (lm.get_end_error() < min_error)
                {
                    min_error=lm.get_end_error();
                    argminx=x;
                    vcl_cout<<":"<<theta;
                }
            }
        }
    }
    //lm.minimize(x);
    //vnl_matrix<double> cv=lm.get_JtJ();

    vcl_cout<<"\n Solution: "
            <<argminx[0]<<','
            <<argminx[1]<<','
            <<argminx[2]<<','
            <<argminx[3]<<','
            <<argminx[4] <<'\n'
            <<"St Error "<<min_error<<vcl_endl;

    int a;vcl_cin>>a;
}

TESTMAIN( test_phongs_model_est );
