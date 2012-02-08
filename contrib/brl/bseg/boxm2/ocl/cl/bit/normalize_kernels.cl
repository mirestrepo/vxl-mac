#if NVIDIA
 #pragma OPENCL EXTENSION cl_khr_gl_sharing : enable
#endif

#ifdef RENDER
__kernel void normalize_render_kernel(__global float * exp_img, 
                                      __global float* vis_img, 
                                      __global uint4* imgdims)
{
    int i=0,j=0;
    i=get_global_id(0);
    j=get_global_id(1);
    int imindex=j*get_global_size(0)+i;
   
    // check to see if the thread corresponds to an actual pixel as in some
    // cases #of threads will be more than the pixels.
    if (i>=(*imgdims).z || j>=(*imgdims).w) 
        return;

    //normalize image with respect to visibility
    float vis   = vis_img[imindex];
    exp_img[imindex] = exp_img[imindex] + (vis*0.5f);
}

__kernel void normalize_render_rgb_kernel(__global float4* exp_img, 
                                          __global float* vis_img, 
                                          __global uint4* imgdims)
{
    int i=0,j=0;
    i=get_global_id(0);
    j=get_global_id(1);
    int imindex=j*get_global_size(0)+i;
   
    // check to see if the thread corresponds to an actual pixel as in some
    // cases #of threads will be more than the pixels.
    if (i>=(*imgdims).z || j>=(*imgdims).w) 
        return;

    //normalize image with respect to visibility
    float vis   = vis_img[imindex];
    exp_img[imindex] = exp_img[imindex]+ (vis*0.5f);
}

#endif

#ifdef RENDER_NAA
__kernel void normalize_render_kernel(__global float * exp_img, 
                                      __global float* vis_img, 
                                      __global uint4* imgdims,
                                      __global float* background_radiance)
{
    int i=0,j=0;
    i=get_global_id(0);
    j=get_global_id(1);
    int imindex=j*get_global_size(0)+i;
   
    // check to see if the thread corresponds to an actual pixel as in some
    // cases #of threads will be more than the pixels.
    if (i>=(*imgdims).z || j>=(*imgdims).w) 
        return;

    //normalize image with respect to visibility
    float vis   = vis_img[imindex];
    exp_img[imindex] = exp_img[imindex] + (vis*(*background_radiance));
}
#endif

#ifdef RENDER_DEPTH
__kernel void normalize_render_depth_kernel(__global float * exp_img,
                                            __global float * var_img,
                                            __global float* prob_img, 
                                            __global uint4* imgdims)
{
    int i=0,j=0;
    i=get_global_id(0);
    j=get_global_id(1);
    int imindex=j*get_global_size(0)+i;
   
    // check to see if the thread corresponds to an actual pixel as in some
    // cases #of threads will be more than the pixels.
    if (i>=(*imgdims).z || j>=(*imgdims).w) 
        return;

    //normalize image with respect to visibility
    float prob   = prob_img[imindex];

    float mean   = exp_img[imindex]/prob;

    float var    = var_img[imindex]/prob -mean*mean;
    exp_img[imindex]=mean;
    var_img[imindex]=var;
}
#endif

#ifdef NORMALIZE_RENDER_GL
__kernel void normalize_render_kernel_gl(__global uint * exp_img, 
                                         __global float* vis_img, 
                                         __global uint4* imgdims)
{
    int i=0,j=0;
    i=get_global_id(0);
    j=get_global_id(1);
    int imindex=j*get_global_size(0)+i;

    // check to see if the thread corresponds to an actual pixel as in some
    // cases #of threads will be more than the pixels.
    if (i<(*imgdims).x || j<(*imgdims).y|| i>=(*imgdims).z || j>=(*imgdims).w) 
        return;

    float intensity  = as_float(exp_img[imindex]);
    float vis = vis_img[imindex];
    intensity += vis*.5f;
    exp_img[imindex] =(rgbaFloatToInt((float4) intensity));//(intensity-*min_i)/range) ;
}

__kernel void normalize_render_kernel_rgb_gl( __global float4* exp_img, 
                                              __global float* vis_img, 
                                              __global uint4* imgdims,
                                              __global uint*  gl_im, 
                                              __global bool*  is_bw)
{
    int i=0,j=0;
    i=get_global_id(0);
    j=get_global_id(1);
    int imindex=j*get_global_size(0)+i;

    // check to see if the thread corresponds to an actual pixel as in some
    // cases #of threads will be more than the pixels.
    if (i<(*imgdims).x || j<(*imgdims).y|| i>=(*imgdims).z || j>=(*imgdims).w) 
        return;

    float vis = vis_img[imindex];
    float4 intensity = exp_img[imindex] + (vis*0.5f);

    if( *is_bw ) 
      gl_im[imindex]   = rgbaFloatToInt(intensity);//(intensity-*min_i)/range) ;
    else 
      gl_im[imindex]   = rgbaFloatToInt( (float4) intensity.x );//(intensity-*min_i)/range) ;
}
#endif

#ifdef RENDER_GL
__kernel void render_kernel_gl(__constant float *min_i,
                               __constant float *max_i,
                               __constant float *tf,
                               __global float   *vis_img,
                               __global float   *exp_img,                               
                               __global uint    *out_img,
                               __global uint4   *imgdims)
{

    
    int i=0,j=0;
    i=get_global_id(0);
    j=get_global_id(1);
    int imindex=j*get_global_size(0)+i;

    // check to see if the thread corresponds to an actual pixel as in some
    // cases #of threads will be more than the pixels.
    if (i<(*imgdims).x || j<(*imgdims).y|| i>=(*imgdims).z || j>=(*imgdims).w) 
        return;

    
    float intensity  = exp_img[imindex];
    float range=(*max_i-*min_i);
    //intensity+=(vis_img[imindex]*((*max_i+*min_i)/2));
    //intensity=clamp(intensity,*mini,*maxi);
    int index=(int)max(floor((intensity-*min_i)/range*255.0f),0.0f);
    out_img[imindex] =rgbaFloatToInt((float4 )tf[index]);//(intensity-*min_i)/range) ;
}

__kernel void render_kernel_rgb_gl(__constant float *min_i,
                                   __constant float *max_i,
                                   __constant float *tf,
                                   __global float   *vis_img,
                                   __global float4  *exp_img,                               
                                   __global uint    *out_img,
                                   __global uint4   *imgdims)
{

    
    int i=0,j=0;
    i=get_global_id(0);
    j=get_global_id(1);
    int imindex=j*get_global_size(0)+i;

    // check to see if the thread corresponds to an actual pixel as in some
    // cases #of threads will be more than the pixels.
    if (i<(*imgdims).x || j<(*imgdims).y|| i>=(*imgdims).z || j>=(*imgdims).w) 
        return;

    float4 intensity  = exp_img[imindex];
    
    //normalize
    intensity = intensity + vis_img[imindex] * 0.5f; 

    //write to gl image
    out_img[imindex] = rgbaFloatToInt(intensity); 
}

#endif

#ifdef CHANGE
__kernel void normalize_change_kernel(__global float* exp_img /* background probability density*/ , 
                                      __global float* prob_exp_img ,
                                      __global float* vis_img, 
                                      __global uint4* imgdims, 
                                      __global int  * rbelief)
{
    int i=0,j=0;
    i=get_global_id(0);
    j=get_global_id(1);
    int imindex=j*get_global_size(0)+i;
    
    // check to see if the thread corresponds to an actual pixel as in some
    // cases #of threads will be more than the pixels.
    if (i>=(*imgdims).z || j>=(*imgdims).w) {
        return;
    }
    
    //get vis, if it's greater than some threshold, we don't have enough information
    float vis   = vis_img[imindex];
    if(vis > .2f) {
      exp_img[imindex] = 0.0f; 
      return; 
    }
    
    float prob_int = exp_img[imindex] ; 
    //prob_int+=vis;

    float prob_exp_int = prob_exp_img[imindex];
    //prob_exp_int+=vis;

    //compute foreground belief
    if( *rbelief == 0 ) {
      float fgbelief   = 1.0f/(1.0f+prob_int);
      exp_img[imindex] = fgbelief; 
    }
    else {
      float fgbelief   = 1.0f/(1.0f+prob_int); // - 0.5f*min(prob_int,1/prob_int);
      float raybelief  = prob_exp_int/(1.0f+prob_exp_int); //-0.5*min(prob_exp_int,1/prob_exp_int);
      
      //thresh
      exp_img[imindex] = raybelief*fgbelief; //(raybelief > .25) ? fgbelief : 0.0f; 
    }
}
#endif
#ifdef PROB_IMAGE
__kernel void normalize_probability_image_kernel(__global float * prob_img ,
                                      __global float* vis_img, 
                                      __global uint4* imgdims)
{
    int i=0,j=0;
    i=get_global_id(0);
    j=get_global_id(1);


    int imindex=j*get_global_size(0)+i;
    // check to see if the thread corresponds to an actual pixel as in some
    // cases #of threads will be more than the pixels.
    if (i>=(*imgdims).z || j>=(*imgdims).w) {
        return;
    }
    float vis   = vis_img[imindex];


    
    float prob = prob_img[imindex] +vis; 
    float raybelief=prob/(1.0f+prob)-0.5*min(prob,1/prob);
    prob_img[imindex]=raybelief;
}
#endif
