// This is gel/vifa/tests/test_region_proc.cxx
#include <vcl_ostream.h>
#include <vcl_ostream.h>
#include <vgl/vgl_vector_2d.h>
#include <testlib/testlib_test.h>
#include <sdet/sdet_detector_params.h>
#include <sdet/sdet_region_proc.h>
#include <sdet/sdet_region_proc_params.h>
#include <vsol/vsol_curve_2d.h>
#include <vsol/vsol_curve_2d_sptr.h>
#include <vtol/vtol_edge_2d.h>
#include <vifa/vifa_int_face_attr.h>
#include <vifa/vifa_typedefs.h>
#include <vil/vil_image_view.h>
#include <vil1/vil1_image.h>
#include <vil1/vil1_image_as.h>
#include <vil1/vil1_load.h>
#include <vil1/vil1_save.h>
#include <vil1/vil1_vil.h>


static void test_region_proc()
{
	// Get the image
	vil1_image	test_img_raw = vil1_load("testimg.tif");

	if (test_img_raw)
	{
		// Convert the image to greyscale
		vil1_image				test_img = vil1_image_as_byte(test_img_raw);

		// Get the detector & region processor parameters
		sdet_detector_params	dp;
		sdet_region_proc_params	rpp(dp, true, true, 1);
		vcl_cout << rpp;

		// Instantiate a region processor & attach the test image
		sdet_region_proc		rp(rpp);
		rp.set_image(test_img);

		// Segment the image
		rp.extract_regions();

		// Get the intensity region list & debug output images
		iface_list&				region_list = rp.get_regions();
		vil1_image				edge_img = rp.get_edge_image();
		vil1_image				res_img = rp.get_residual_image();

		// Save the debug output images for external viewing
		vil1_save(edge_img, "testimg_edges.tif");
		vil1_save(res_img, "testimg_residual.tif");

		// Dump the intensity regions
		vcl_cout << region_list.size() << " intensity faces found:" << vcl_endl;
		iface_iterator	ifi = region_list.begin();
		int				i = 1;
		for (; ifi != region_list.end(); ifi++, i++)
		{
			vdgl_intensity_face_sptr	face = (*ifi);

			vcl_cout << "  Intensity Face #" << i << ": " << (*face);

			one_chain_list*				ocl = face->one_chains();

			if (ocl)
			{
				one_chain_list::iterator	ocli = ocl->begin();
				int							j = 1;
				double						perim1 = 0.0;
				double						perim2 = 0.0;
				for (; ocli != ocl->end(); ocli++, j++)
				{
					vtol_one_chain_sptr	oc = (*ocli);

					vcl_cout << "    One Chain #" << j << ": " << (*oc);

					topology_list*		tl = oc->inferiors();

					if (tl)
					{
						topology_list::const_iterator	tli = tl->begin();
						int								k = 1;
						for (; tli != tl->end(); tli++, k++)
						{
							vcl_cout << "      Edge #" << k << ": ";
							(*tli)->describe(vcl_cout, 6);

							vtol_edge_2d*	e;
							if ((*tli)->topology_type() ==
								vtol_topology_object::EDGE)
							{
								e = (*tli)->cast_to_edge()->cast_to_edge_2d();
							}

							if (e)
							{
								vtol_vertex_2d*			p1 =
									e->v1()->cast_to_vertex_2d();
								vtol_vertex_2d*			p2 =
									e->v2()->cast_to_vertex_2d();
								vgl_vector_2d<double>	v(p2->x() - p1->x(),
														  p2->y() - p1->y());
								double	l1 = v.length();
								double	l2 = e->curve()->length();

								vcl_cout << "Length: " << l1 << " (E)  "
									<< l2 << " (C)" << vcl_endl;

								perim1 += l1;
								perim2 += l2;
							}
						}
					}

					vcl_cout << "Perimeter (Edge-based): " << perim1
						<< vcl_endl;
					vcl_cout << "Perimeter (Curve-based): " << perim2
						<< vcl_endl;
					oc->describe_directions(vcl_cout, 4);
				}

				delete ocl;
			}
		}
	}
}


TESTMAIN(test_region_proc);
