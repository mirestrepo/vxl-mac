//:
// \file
// \author Ozge C Ozcanli (ozge@lems.brown.edu)
// \date 10/16/08
//
//

#include <rec/bvxm_part_hierarchy.h>
#include <rec/bvxm_part_base.h>

#include <vil/vil_image_view.h>

//: generate a map from the activated parts
void bvxm_part_hierarchy::generate_map(vcl_vector<bvxm_part_instance_sptr>& extracted_parts, vil_image_view<float>& map, vil_image_view<unsigned>& type_map)
{
  unsigned ni = map.ni();
  unsigned nj = map.nj();
  map.fill(0.0f);
  type_map.fill(0);
  for (unsigned i = 0; i < extracted_parts.size(); i++) {
    bvxm_part_instance_sptr p = extracted_parts[i];
    unsigned ii = (unsigned)p->x_;
    unsigned jj = (unsigned)p->y_;
    if (ii > 0 && ii < ni && jj > 0 && jj < nj) {
      map(ii, jj, 0) = p->strength_;
      type_map(ii, jj, 0) = p->type_;
    }
  }
}

void bvxm_part_hierarchy::generate_map(vcl_vector<bvxm_part_instance_sptr>& extracted_parts, vcl_vector<vcl_vector<bvxm_part_instance_sptr> >& map)
{
  unsigned ni = map.size();
  unsigned nj = map[0].size();
  for (unsigned i = 0; i < ni; i++)
    for (unsigned j = 0; j < nj; j++)
      map[i][j] = 0;

  for (unsigned i = 0; i < extracted_parts.size(); i++) {
    bvxm_part_instance_sptr p = extracted_parts[i];
    unsigned ii = (unsigned)p->x_;
    unsigned jj = (unsigned)p->y_;
    if (ii > 0 && ii < ni && jj > 0 && jj < nj) {
      map[ii][jj] = p;
    }
  }
}

//: generate a float map with normalized strengths and receptive fields marked
void
bvxm_part_hierarchy::generate_output_map(vcl_vector<bvxm_part_instance_sptr>& extracted_parts, vil_image_view<float>& map)
{
  map.fill(0.0f);

  float max = -1e99f;
  for (unsigned i = 0; i < extracted_parts.size(); ++i) {
    if (extracted_parts[i]->strength_ > max)
      max = extracted_parts[i]->strength_;
  }

  for (unsigned i = 0; i < extracted_parts.size(); ++i) {
    bvxm_part_instance_sptr p = extracted_parts[i];
    p->mark_receptive_field(map, p->strength_/max);
  }
}

//: output_img needs to have 3 planes
void
bvxm_part_hierarchy::generate_output_img(vcl_vector<bvxm_part_instance_sptr>& extracted_parts,
                                          vil_image_view<vxl_byte>& input_img,
                                          vil_image_view<vxl_byte>& output_img)
{
  unsigned ni = input_img.ni();
  unsigned nj = input_img.nj();
  output_img.fill(0);

  vil_image_view<float> map(ni, nj);
  generate_output_map(extracted_parts, map);
  for (unsigned i = 0; i < ni; i++)
    for (unsigned j = 0; j < nj; j++) {
      output_img(i,j,0) = input_img(i,j);
      output_img(i,j,1) = input_img(i,j);
      output_img(i,j,2) = (vxl_byte)(map(i,j)*255);
    }
}

bvxm_part_base_sptr bvxm_part_hierarchy::get_node(unsigned layer, unsigned type)
{
  for (vertex_iterator it = this->vertices_begin(); it != this->vertices_end(); it++) {
    if ((*it)->layer_ == layer && (*it)->type_ == type)
      return *it;
  }
  return 0;
}

unsigned bvxm_part_hierarchy::highest_layer_id()
{
  unsigned highest = 0;
  for (vertex_iterator it = this->vertices_begin(); it != this->vertices_end(); it++) {
    if ((*it)->layer_ > highest)
      highest = (*it)->layer_;
  }
  return highest;
}

// check for existence of upper_p with central_p as its central part and map will tell if all the other parts exist
// map is the activation map of the parts at the layer of central_p
bvxm_part_instance_sptr bvxm_part_hierarchy::exists(bvxm_part_base_sptr upper_p,
                                                      bvxm_part_instance_sptr central_p,
                                                      vil_image_view<float>& map,
                                                      vil_image_view<unsigned>& type_map,
                                                      vcl_vector<vcl_vector<bvxm_part_instance_sptr> >& part_map,
                                                      float det_threshold)
{
  unsigned ni = map.ni();
  unsigned nj = map.nj();

  //: first check if types and layers of central_p instance matches with upper_p's info
  if (upper_p->central_part()->type_ != central_p->type_ || upper_p->layer_ != central_p->layer_ + 1) {
    vcl_cout << "central_p instance passed is not compatible with the upper layer part passes\n";
    return 0;
  }

  bvxm_part_instance_sptr pi = new bvxm_part_instance(upper_p->layer_, upper_p->type_, bvxm_part_instance_kind::COMPOSED, central_p->x_, central_p->y_, 0.0f);
  bvxm_hierarchy_edge_sptr e1 = new bvxm_hierarchy_edge(pi->cast_to_base(), central_p->cast_to_base());
  pi->add_outgoing_edge(e1);

  //: now for each other part of upper_p, check whether they exist in the map
  float cx = central_p->x_; float cy = central_p->y_;
  edge_iterator eit = upper_p->out_edges_begin();
  eit++;  // skip the central part
  float strength = 1.0f;
  for ( ; eit != upper_p->out_edges_end(); eit++)
  {
    //int mx = (int)vcl_floor(cx+(*eit)->mean().get(0)+0.5);
    //int my = (int)vcl_floor(cy+(*eit)->mean().get(1)+0.5);
    //int rad = (int)vcl_ceil(vcl_sqrt((*eit)->var())+3);
    int mx = (int)vcl_floor(cx+0.5);  // try all locations around center within dist+variance_dist radius
    int my = (int)vcl_floor(cy+0.5);
    int rad = (int)vcl_ceil((*eit)->mean_dist()+vcl_sqrt((*eit)->var_dist())+3);
    int si = mx - rad;  si = si < 0 ? 0 : si;
    int upper_i = mx + rad; upper_i = upper_i > (int)ni ? ni : upper_i;
    int sj = my - rad;  sj = sj < 0 ? 0 : sj;
    int upper_j = my + rad; upper_j = upper_j > (int)nj ? nj : upper_j;
    float best_fit = 0.0f;
    float best_fit_str = 1.0f;
    bvxm_part_instance_sptr best_part;
    for (int i = si ; i < upper_i; i++) {
      for (int j = sj ; j < upper_j; j++) {
        if (map(i, j) > det_threshold && type_map(i, j) == (*eit)->target()->type_) {
          vnl_vector_fixed<float, 2> v((float)i-cx, (float)j-cy);
          float dist, angle;
          (*eit)->calculate_dist_angle(central_p, v, dist, angle);
          float str = (*eit)->prob_density(dist, angle);
          if (str < det_threshold)
            continue;
          if (best_fit < str) {
            best_fit = str;
            best_fit_str = map(i,j);
            best_part = part_map[i][j];
          }
        }
      }
    }
    if (best_fit <= 0)
      return 0;  // this sub-part not found
    strength *= best_fit*best_fit_str;
    if (best_part) {
      bvxm_hierarchy_edge_sptr e2 = new bvxm_hierarchy_edge(pi->cast_to_base(), best_part->cast_to_base());
      pi->add_outgoing_edge(e2);
    }
  }
  strength *= central_p->strength_;

  //: if all of them have been detected then declare existence at the central parts location
  pi->strength_ = strength;

  return pi;
}

//: given a set of detected lower level parts, create a set of instance detections for one layer above in the hierarchy
void bvxm_part_hierarchy::extract_upper_layer(vcl_vector<bvxm_part_instance_sptr>& extracted_parts,
                                               unsigned ni, unsigned nj,
                                               vcl_vector<bvxm_part_instance_sptr>& extracted_upper_parts)
{
  vil_image_view<float> map(ni, nj, 1);  // the second plane will hold the types of the primitives
  vil_image_view<unsigned> type_map(ni, nj, 1);  // the second plane will hold the types of the primitives
  generate_map(extracted_parts, map, type_map);

  vcl_vector<bvxm_part_instance_sptr> dummy(nj, 0);
  vcl_vector<vcl_vector<bvxm_part_instance_sptr> > part_map(ni, dummy);
  generate_map(extracted_parts, part_map);

  //: we'll generate a list of instantiations for each part in the upper parts of the hierarchy
  //vcl_map<vcl_pair<unsigned, unsigned>, vcl_vector<bvxm_part_instance_sptr> > instantiations;

  //: for each detected part, check for the existence of each upper layer part that uses it as a central part
  for (unsigned i = 0; i < extracted_parts.size(); i++)
  {
    bvxm_part_instance_sptr p = extracted_parts[i];
    //: find this type in the primitive layer of the hierarchy
    bvxm_part_base_sptr hp = this->get_node(p->layer_, p->type_);
    if (!hp)
      continue;

    //: find the all the upper layer parts that use hp as a central part
    //  check the incoming edges of hp
    for (edge_iterator eit = hp->in_edges_begin(); eit != hp->in_edges_end(); eit++) {
      if (hp == (*eit)->source()->central_part()) {
        bvxm_part_base_sptr hp_upper = (*eit)->source();

        // now check for the existence of other primitives wrt to the central part and initiate an instance of it if so
        bvxm_part_instance_sptr hp_upper_instance = exists(hp_upper, p, map, type_map, part_map, hp_upper->detection_threshold_); // p will be its central part and map will tell if all the other parts exist
        if (!hp_upper_instance)
          continue;
        if (hp_upper_instance->strength_ > hp_upper->detection_threshold_)
          extracted_upper_parts.push_back(hp_upper_instance);
      }
    }
  }
}
