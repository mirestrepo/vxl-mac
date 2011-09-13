#ifndef boxm2_compute_nonsurface_histogram_functor_h_
#define boxm2_compute_nonsurface_histogram_functor_h_
//:
// \file

#include <boxm2/cpp/algo/boxm2_cast_ray_function.h>
#include <boxm2/cpp/algo/boxm2_mog3_grey_processor.h>
#include <boxm2/io/boxm2_stream_cache.h>
#include <bsta/algo/bsta_sigma_normalizer.h>

class boxm2_compute_nonsurface_histogram_functor
{
  public:
    typedef boxm2_data_traits<BOXM2_AUX0>::datatype aux0_datatype;
    typedef boxm2_data_traits<BOXM2_AUX1>::datatype aux1_datatype;
    typedef boxm2_data_traits<BOXM2_AUX2>::datatype aux2_datatype;

    //: "default" constructor
    boxm2_compute_nonsurface_histogram_functor() {}

    bool init_data(boxm2_data_base * histo_data,
                   boxm2_data_base * entropy_histo_data,
                   boxm2_stream_cache_sptr str_cache)
    {
        histo_data_=new boxm2_data<BOXM2_MOG3_GREY>(histo_data->data_buffer(),
                                                    histo_data->buffer_length(),
                                                    histo_data->block_id());

        entropy_histo_data_=new boxm2_data<BOXM2_AUX0>(entropy_histo_data->data_buffer(),
                                                       entropy_histo_data->buffer_length(),
                                                       entropy_histo_data->block_id());
        str_cache_ = str_cache;
        id_ = histo_data->block_id();
        return true;
    }

    inline bool process_cell(int index)
    {
        boxm2_data<BOXM2_MOG3_GREY>::datatype & histo=histo_data_->data()[index];
        boxm2_data<BOXM2_AUX0>::datatype            & entropy_histo=entropy_histo_data_->data()[index];

        vcl_vector<aux0_datatype>  out0   = str_cache_->get_next<BOXM2_AUX0>(id_, index);
        vcl_vector<aux1_datatype>  out1   = str_cache_->get_next<BOXM2_AUX1>(id_, index);
        vcl_vector<aux2_datatype>  out2   = str_cache_->get_next<BOXM2_AUX2>(id_, index);
        unsigned nobs = (unsigned)out0.size();

        vcl_vector<float> temp_histogram(8,0.125f);
        for (unsigned i =0;i<histo.size();i++) histo[i] = 1.0f;

        vcl_vector<float> Iobs;
        vcl_vector<float> vis;
        for (unsigned i = 0; i < nobs; i++)
        {
            if (out0[i]>1e-10f)
            {
                float mean_obs = out1[i]/out0[i];
                float vis_i    = out2[i]/out0[i]; // mean vis

                Iobs.push_back(mean_obs);
                vis.push_back(vis_i);
            }
        }

        float sum = 1.0;
        for (unsigned i=0;i<Iobs.size();i++)
        {
            unsigned index = i;
            if (i == Iobs.size()-1)
                index =0;
            else
                index = i+1;
            float gradI=vcl_fabs(Iobs[i]-Iobs[index]);

            int bin_index  = (int) vcl_floor(gradI*8);
            bin_index = bin_index>7 ? 7:bin_index;
            temp_histogram[bin_index] += (vis[i]+vis[index])/2;
            sum+= (vis[i]+vis[index])/2;
        }
        for (unsigned i =0; i < 8;i++) temp_histogram[i] /= sum;

        // Normalize histogram
        for (unsigned i = 0; i < temp_histogram.size(); i++)
                histo[i] = (unsigned char)vcl_floor(temp_histogram[i] *255.0f ) ;

        entropy_histo  =0.0;
        for (unsigned int i = 0; i<8; ++i)
        {
            double pi = (double)(histo[i])/255.0;
            if (pi>0)
                entropy_histo -= pi*vcl_log(static_cast<double>(pi));
        }
        entropy_histo *= vnl_math::log2e;


        return true;
    }

  private:
    boxm2_data<BOXM2_MOG3_GREY>* histo_data_;
    boxm2_data<BOXM2_AUX0>* entropy_histo_data_;
    boxm2_stream_cache_sptr str_cache_;
    boxm2_block_id id_;
};


#endif // boxm2_compute_nonsurface_histogram_functor_h_
