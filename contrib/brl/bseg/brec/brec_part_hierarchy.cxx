//:
// \file
// \author Ozge C. Ozcanli (ozge at lems dot brown dot edu)
// \date Oct. 16, 2008
//

#include "brec_part_hierarchy.h"
#include "brec_part_base.h"
#include "brec_part_gaussian_sptr.h"
#include "brec_part_gaussian.h"

#include <vil/vil_image_view.h>

#include <bxml/bxml_read.h>
#include <bxml/bxml_write.h>
#include <bxml/bxml_find.h>

//: generate a map from the activated parts
void brec_part_hierarchy::generate_map(vcl_vector<brec_part_instance_sptr>& extracted_parts, vil_image_view<float>& map, vil_image_view<unsigned>& type_map)
{
  unsigned ni = map.ni();
  unsigned nj = map.nj();
  map.fill(0.0f);
  type_map.fill(0);
  for (unsigned i = 0; i < extracted_parts.size(); i++) {
    brec_part_instance_sptr p = extracted_parts[i];
    unsigned ii = (unsigned)p->x_;
    unsigned jj = (unsigned)p->y_;
    if (ii > 0 && ii < ni && jj > 0 && jj < nj) {
      map(ii, jj, 0) = p->strength_;
      type_map(ii, jj, 0) = p->type_;
    }
  }
}

void brec_part_hierarchy::generate_map(vcl_vector<brec_part_instance_sptr>& extracted_parts, vcl_vector<vcl_vector<brec_part_instance_sptr> >& map)
{
  unsigned ni = map.size();
  unsigned nj = map[0].size();
  for (unsigned i = 0; i < ni; i++)
    for (unsigned j = 0; j < nj; j++)
      map[i][j] = 0;

  for (unsigned i = 0; i < extracted_parts.size(); i++) {
    brec_part_instance_sptr p = extracted_parts[i];
    unsigned ii = (unsigned)p->x_;
    unsigned jj = (unsigned)p->y_;
    if (ii > 0 && ii < ni && jj > 0 && jj < nj) {
      map[ii][jj] = p;
    }
  }
}

//: generate a float map with normalized strengths and receptive fields marked
void
brec_part_hierarchy::generate_output_map(vcl_vector<brec_part_instance_sptr>& extracted_parts, vil_image_view<float>& map)
{
  map.fill(0.0f);

  float max = -1e38f; // min float
  for (unsigned i = 0; i < extracted_parts.size(); ++i) {
    if (extracted_parts[i]->strength_ > max)
      max = extracted_parts[i]->strength_;
  }

  for (unsigned i = 0; i < extracted_parts.size(); ++i) {
    brec_part_instance_sptr p = extracted_parts[i];
    p->mark_receptive_field(map, p->strength_/max);
  }
}

//: generate a float map with strengths and receptive fields marked
//  Return the values as they are
void
brec_part_hierarchy::generate_output_map2(vcl_vector<brec_part_instance_sptr>& extracted_parts, vil_image_view<float>& map)
{
  map.fill(0.0f);

  for (unsigned i = 0; i < extracted_parts.size(); ++i) {
    brec_part_instance_sptr p = extracted_parts[i];
    p->mark_receptive_field(map, p->strength_);
  }
}

//: generate a float map with strengths and receptive fields marked
//  Stretch the values to be used for imaging
void
brec_part_hierarchy::generate_output_map3(vcl_vector<brec_part_instance_sptr>& extracted_parts, vil_image_view<float>& map)
{
  map.fill(0.0f);

  // find the mean value and stretch the values onto [0, mean];
  float mean = 0.0f;
#if 0
  for (unsigned i = 0; i < extracted_parts.size(); ++i) {
    brec_part_instance_sptr p = extracted_parts[i];
    mean += p->strength_;
  }
  mean /= extracted_parts.size();
#endif // 0
  mean = 0.00000005f; // we want to see all the detections, this value is the smallest threshold used to create the ROC

  for (unsigned i = 0; i < extracted_parts.size(); ++i) {
    brec_part_instance_sptr p = extracted_parts[i];
    if (p->strength_ > mean)
      p->mark_receptive_field(map, 1.0f);
    else
      p->mark_receptive_field(map, p->strength_/mean);
  }
}

//: output_img needs to have 3 planes
void
brec_part_hierarchy::generate_output_img(vcl_vector<brec_part_instance_sptr>& extracted_parts,
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

brec_part_base_sptr brec_part_hierarchy::get_node(unsigned layer, unsigned type)
{
  for (vertex_iterator it = this->vertices_begin(); it != this->vertices_end(); it++) {
    if ((*it)->layer_ == layer && (*it)->type_ == type)
      return *it;
  }
  return 0;
}

unsigned brec_part_hierarchy::highest_layer_id()
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
brec_part_instance_sptr brec_part_hierarchy::exists(brec_part_base_sptr upper_p,
                                                    brec_part_instance_sptr central_p,
                                                    vil_image_view<float>& map,
                                                    vil_image_view<unsigned>& type_map,
                                                    vcl_vector<vcl_vector<brec_part_instance_sptr> >& part_map,
                                                    float det_threshold)
{
  unsigned ni = map.ni();
  unsigned nj = map.nj();

  // first check if types and layers of central_p instance matches with upper_p's info
  if (upper_p->central_part()->type_ != central_p->type_ || upper_p->layer_ != central_p->layer_ + 1) {
    vcl_cout << "central_p instance passed is not compatible with the upper layer part passes\n";
    return 0;
  }

  brec_part_instance_sptr pi = new brec_part_instance(upper_p->layer_, upper_p->type_, brec_part_instance_kind::COMPOSED, central_p->x_, central_p->y_, 0.0f);
  brec_hierarchy_edge_sptr e1 = new brec_hierarchy_edge(pi->cast_to_base(), central_p->cast_to_base());
  pi->add_outgoing_edge(e1);

  // now for each other part of upper_p, check whether they exist in the map
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
    brec_part_instance_sptr best_part;
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
      brec_hierarchy_edge_sptr e2 = new brec_hierarchy_edge(pi->cast_to_base(), best_part->cast_to_base());
      pi->add_outgoing_edge(e2);
    }
  }
  strength *= central_p->strength_;

  // if all of them have been detected then declare existence at the central parts location
  pi->strength_ = strength;

  return pi;
}

//: given a set of detected lower level parts, create a set of instance detections for one layer above in the hierarchy
void brec_part_hierarchy::extract_upper_layer(vcl_vector<brec_part_instance_sptr>& extracted_parts,
                                              unsigned ni, unsigned nj,
                                              vcl_vector<brec_part_instance_sptr>& extracted_upper_parts)
{
  vil_image_view<float> map(ni, nj, 1);  // the second plane will hold the types of the primitives
  vil_image_view<unsigned> type_map(ni, nj, 1);  // the second plane will hold the types of the primitives
  generate_map(extracted_parts, map, type_map);

  vcl_vector<brec_part_instance_sptr> dummy(nj, 0);
  vcl_vector<vcl_vector<brec_part_instance_sptr> > part_map(ni, dummy);
  generate_map(extracted_parts, part_map);

  // we'll generate a list of instantiations for each part in the upper parts of the hierarchy
  //vcl_map<vcl_pair<unsigned, unsigned>, vcl_vector<brec_part_instance_sptr> > instantiations;

  // for each detected part, check for the existence of each upper layer part that uses it as a central part
  for (unsigned i = 0; i < extracted_parts.size(); i++)
  {
    brec_part_instance_sptr p = extracted_parts[i];
    // find this type in the primitive layer of the hierarchy
    brec_part_base_sptr hp = this->get_node(p->layer_, p->type_);
    if (!hp)
      continue;

    // find the all the upper layer parts that use hp as a central part
    // check the incoming edges of hp
    for (edge_iterator eit = hp->in_edges_begin(); eit != hp->in_edges_end(); eit++) {
      if (hp == (*eit)->source()->central_part()) {
        brec_part_base_sptr hp_upper = (*eit)->source();

        // now check for the existence of other primitives wrt to the central part and initiate an instance of it if so
        brec_part_instance_sptr hp_upper_instance = exists(hp_upper, p, map, type_map, part_map, hp_upper->detection_threshold_); // p will be its central part and map will tell if all the other parts exist
        if (!hp_upper_instance)
          continue;
        if (hp_upper_instance->strength_ > hp_upper->detection_threshold_)
          extracted_upper_parts.push_back(hp_upper_instance);
      }
    }
  }
}

//: reader adds the edges of each vertex in the order read from the xml file
//  CAUTION: assumes that central part is added prior to other children of a parent part hence its edge is added to the hierarchy before other parts
void brec_part_hierarchy::write_xml(vcl_ostream& os)
{
  bxml_document doc;
  bxml_element * root = new bxml_element("hierarchy");
  doc.set_root_element(root);
  root->append_text("\n");
  root->set_attribute("name", name_);
  root->set_attribute("model_dir", model_dir_);

  bxml_element * prims = new bxml_element("primitive_instances");
  prims->append_text("\n");
  root->append_data(prims);
  root->append_text("\n");
  for (unsigned i=0; i<dummy_primitive_instances_.size(); ++i) {
    bxml_data_sptr ins = dummy_primitive_instances_[i]->xml_element();
    prims->append_text("  ");
    prims->append_data(ins);
    prims->append_text("\n");
  }

  bxml_element *vertices = new bxml_element("vertices");
  vertices->append_text("\n");
  root->append_data(vertices);
  root->append_text("\n");

  vcl_map<brec_part_base_sptr, unsigned> vert_map;
  unsigned id = 0;
  for (vertex_iterator it = this->vertices_begin(); it != this->vertices_end(); it++) {
    bxml_data_sptr v = (*it)->xml_element();
    vertices->append_text("  ");
    vertices->append_data(v);
    vertices->append_text("\n");
    vert_map[(*it)] = id;
    id++;
  }

  bxml_element *edges = new bxml_element("edges");
  edges->append_text("\n");
  root->append_data(edges);
  root->append_text("\n");

  for (edge_iterator it = this->edges_begin(); it != this->edges_end(); it++) {
    bxml_data_sptr e = (*it)->xml_element();
    edges->append_text("  ");
    edges->append_data(e);
    edges->append_text("\n");

    bxml_element *e_con = new bxml_element("connectivity");
    e_con->set_attribute("source", vert_map[(*it)->source()]);
    e_con->set_attribute("target", vert_map[(*it)->target()]);
    e_con->append_text("\n");
    ((bxml_element*)e.ptr())->append_data(e_con);
    ((bxml_element*)e.ptr())->append_text("\n");
  }

  bxml_write(os,doc);
}

bool brec_part_hierarchy::read_xml(vcl_istream& is)
{
  bxml_document doc = bxml_read(is);
  bxml_element query("hierarchy");

  bxml_data_sptr hierarchy_root = bxml_find_by_name(doc.root_element(), query);

  if (!hierarchy_root) {
    vcl_cout << "brec_part_hierarchy::read_xml() - could not find the main node with name hierarchy!\n";
    return false;
  }

  bxml_element* re = (bxml_element*)hierarchy_root.ptr();
  re->get_attribute("name", name_);
  vcl_cout << "reading hierarchy with name: " << name_ << vcl_endl;
  re->get_attribute("model_dir", model_dir_);

  bxml_element query2("primitive_instances");
  bxml_data_sptr prims_root = bxml_find_by_name(hierarchy_root, query2);

  if (!prims_root || prims_root->type() != bxml_data::ELEMENT) {
    vcl_cout << "brec_part_hierarchy::read_xml() - could not find the primitive instances node!\n";
    return false;
  }

  bxml_element* pe = (bxml_element*)prims_root.ptr();

  for (bxml_element::const_data_iterator it = pe->data_begin(); it != pe->data_end(); it++) {
    if ((*it)->type() != bxml_data::ELEMENT)
      continue;

    brec_part_instance_sptr ins = new brec_part_instance();
    if (!ins->xml_parse_element(*it))
      return false;

    switch (ins->kind_) {
      case brec_part_instance_kind::GAUSSIAN :
      {
        brec_part_gaussian_sptr g_p = new brec_part_gaussian();
        g_p->xml_parse_element(*it);
        dummy_primitive_instances_.push_back(g_p->cast_to_instance());
        break;
      }
      default: {
        vcl_cout << "brec_part_hierarchy::read_xml() - primitive part kind: " << ins->kind_ << " not recognized by the parser!\n";
        return false;
      }
    }
  }

  bxml_element query3("vertices");
  bxml_data_sptr vert_root = bxml_find_by_name(hierarchy_root, query3);
  if (!vert_root || vert_root->type() != bxml_data::ELEMENT) {
    vcl_cout << "brec_part_hierarchy::read_xml() - could not find the vertices node!\n";
    return false;
  }

  vcl_map<unsigned, brec_part_base_sptr> vert_map;
  unsigned id = 0;
  pe = (bxml_element*)vert_root.ptr();
  for (bxml_element::const_data_iterator it = pe->data_begin(); it != pe->data_end(); it++) {
    if ((*it)->type() != bxml_data::ELEMENT)
      continue;

    brec_part_base_sptr p = new brec_part_base();
    if (!p->xml_parse_element(*it))
      return false;

    vert_map[id] = p;
    this->add_vertex(p);
    id++;
  }

  bxml_element query4("edges");
  bxml_data_sptr e_root = bxml_find_by_name(hierarchy_root, query4);
  if (!e_root || e_root->type() != bxml_data::ELEMENT) {
    vcl_cout << "brec_part_hierarchy::read_xml() - could not find the edges node!\n";
    return false;
  }

  pe = (bxml_element*)e_root.ptr();
  for (bxml_element::const_data_iterator it = pe->data_begin(); it != pe->data_end(); it++) {
    if ((*it)->type() != bxml_data::ELEMENT)
      continue;

    brec_hierarchy_edge_sptr e = new brec_hierarchy_edge();
    if (!e->xml_parse_element(*it))
      return false;

    bxml_element query("connectivity");
    bxml_data_sptr r = bxml_find_by_name((*it), query);
    if (!r || r->type() != bxml_data::ELEMENT) {
      vcl_cout << "brec_part_hierarchy::read_xml() - could not find the edge node: connectivity!\n";
      return false;
    }

    unsigned source_id, target_id;
    ((bxml_element*)(r.ptr()))->get_attribute("source", source_id);
    ((bxml_element*)(r.ptr()))->get_attribute("target", target_id);
    e->set_source(vert_map[source_id]);
    e->set_target(vert_map[target_id]);
    vert_map[source_id]->add_outgoing_edge(e);
    vert_map[target_id]->add_incoming_edge(e);
    this->add_edge_no_check(e);
  }

  return true;
}

//: Binary io, NOT IMPLEMENTED, signatures defined to use brec_part_hierarchy as a brdb_value
void vsl_b_write(vsl_b_ostream & os, brec_part_hierarchy const & /*ph*/)
{
  os << "vsl_b_write() -- Binary io, NOT IMPLEMENTED, signatures defined to use brec_part_hierarchy as a brdb_value\n";
  if (&os != &vcl_cerr)
    vcl_cerr << "vsl_b_write() -- Binary io, NOT IMPLEMENTED, signatures defined to use brec_part_hierarchy as a brdb_value\n";
  return;
}

void vsl_b_read(vsl_b_istream & /*is*/, brec_part_hierarchy & /*ph*/)
{
  vcl_cerr << "vsl_b_read() -- Binary io, NOT IMPLEMENTED, signatures defined to use brec_part_hierarchy as a brdb_value\n";
  return;
}

void vsl_b_read(vsl_b_istream& is, brec_part_hierarchy* ph)
{
  delete ph;
  bool not_null_ptr;
  vsl_b_read(is, not_null_ptr);
  if (not_null_ptr)
  {
    ph = new brec_part_hierarchy();
    vsl_b_read(is, *ph);
  }
  else
    ph = 0;
}

void vsl_b_write(vsl_b_ostream& os, const brec_part_hierarchy* &ph)
{
  if (ph==0)
  {
    vsl_b_write(os, false); // Indicate null pointer stored
  }
  else
  {
    vsl_b_write(os,true); // Indicate non-null pointer stored
    vsl_b_write(os,*ph);
  }
}

