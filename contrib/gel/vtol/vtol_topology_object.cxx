#include <vtol/vtol_topology_object.h>
#include <vtol/vtol_topology_cache.h>
#include <vcl_cassert.h>

//***************************************************************************
// Initialization
//***************************************************************************

//---------------------------------------------------------------------------
// Default constructor
//---------------------------------------------------------------------------
vtol_topology_object::vtol_topology_object(void)
  :_superiors(0),
   _inferiors(0)
{
  inf_sup_cache= new vtol_topology_cache(this);
  touch();
}

//---------------------------------------------------------------------------
// -- Constructor with given sizes for arrays of inferiors and superiors
//---------------------------------------------------------------------------
vtol_topology_object::vtol_topology_object(const int num_inferiors,
                                                 const int num_superiors)
  :_superiors(num_superiors),
   _inferiors(num_inferiors)
{
  inf_sup_cache=new vtol_topology_cache(this);
  touch();
}

//---------------------------------------------------------------------------
// Destructor
//---------------------------------------------------------------------------
vtol_topology_object::~vtol_topology_object()
{
  delete inf_sup_cache;
}

//***************************************************************************
// Replaces dynamic_cast<T>
//***************************************************************************

//---------------------------------------------------------------------------
// -- Return `this' if `this' is a vertex, 0 otherwise
//---------------------------------------------------------------------------
const vtol_vertex *vtol_topology_object::cast_to_vertex(void) const
{
  return 0;
}

//---------------------------------------------------------------------------
// -- Return `this' if `this' is a vertex, 0 otherwise
//---------------------------------------------------------------------------
vtol_vertex *vtol_topology_object::cast_to_vertex(void)
{
  return 0;
}

//---------------------------------------------------------------------------
// -- Return `this' if `this' is a zero_chain, 0 otherwise
//---------------------------------------------------------------------------
const vtol_zero_chain *
vtol_topology_object::cast_to_zero_chain(void) const
{
  return 0;
}

//---------------------------------------------------------------------------
// -- Return `this' if `this' is a zero_chain, 0 otherwise
//---------------------------------------------------------------------------
vtol_zero_chain *vtol_topology_object::cast_to_zero_chain(void)
{
  return 0;
}

//---------------------------------------------------------------------------
// -- Return `this' if `this' is an edge, 0 otherwise
//---------------------------------------------------------------------------
const vtol_edge *vtol_topology_object::cast_to_edge(void) const
{
  return 0;
}

//---------------------------------------------------------------------------
// -- Return `this' if `this' is an edge, 0 otherwise
//---------------------------------------------------------------------------
vtol_edge *vtol_topology_object::cast_to_edge(void)
{
  return 0;
}

//---------------------------------------------------------------------------
// -- Return `this' if `this' is an one_chain, 0 otherwise
//---------------------------------------------------------------------------
const vtol_one_chain *vtol_topology_object::cast_to_one_chain(void) const
{
  return 0;
}

//---------------------------------------------------------------------------
// -- Return `this' if `this' is an one_chain, 0 otherwise
//---------------------------------------------------------------------------
vtol_one_chain *vtol_topology_object::cast_to_one_chain(void)
{
  return 0;
}

//---------------------------------------------------------------------------
// -- Return `this' if `this' is a face, 0 otherwise
//---------------------------------------------------------------------------
const vtol_face *vtol_topology_object::cast_to_face(void) const
{
  return 0;
}

//---------------------------------------------------------------------------
// -- Return `this' if `this' is a face, 0 otherwise
//---------------------------------------------------------------------------
vtol_face *vtol_topology_object::cast_to_face(void)
{
  return 0;
}

//---------------------------------------------------------------------------
// -- Return `this' if `this' is a two_chain, 0 otherwise
//---------------------------------------------------------------------------
const vtol_two_chain *
vtol_topology_object::cast_to_two_chain(void) const
{
  return 0;
}

//---------------------------------------------------------------------------
// -- Return `this' if `this' is a two_chain, 0 otherwise
//---------------------------------------------------------------------------
vtol_two_chain *vtol_topology_object::cast_to_two_chain(void)
{
  return 0;
}

//---------------------------------------------------------------------------
// -- Return `this' if `this' is a block, 0 otherwise
//---------------------------------------------------------------------------
const vtol_block *vtol_topology_object::cast_to_block(void) const
{
  return 0;
}

//---------------------------------------------------------------------------
// -- Return `this' if `this' is a block, 0 otherwise
//---------------------------------------------------------------------------
vtol_block *vtol_topology_object::cast_to_block(void)
{
  return 0;
}

//***************************************************************************
// Status report
//***************************************************************************

//---------------------------------------------------------------------------
// -- Is `inferior' already an inferior of `this' ?
//---------------------------------------------------------------------------
bool
vtol_topology_object::is_inferior(const vtol_topology_object &inferior) const
{
  
  
 
  vcl_vector<vtol_topology_object_ref>::const_iterator i;

  

  for (i=_inferiors.begin(); i!=_inferiors.end(); ++i){

    
    if((*i).ptr()== &inferior){
    
      // if (*(*i) == inferior){

      
      return true;
    }	
  }

  

  return false;
}

//---------------------------------------------------------------------------
// -- Is `superior' already an superior of `this' ?
//---------------------------------------------------------------------------
bool
vtol_topology_object::is_superior(const vtol_topology_object &superior) const
{
 
  
  vcl_list<vtol_topology_object_ref>::const_iterator i;
  for (i=_superiors.begin(); i!=_superiors.end(); ++i){
    // if ( *(*i) == superior){
    
      
    if ((*i).ptr() == &superior){
      return true;
    }
   
  }
  return false;
}

//---------------------------------------------------------------------------
// -- Number of inferiors
//---------------------------------------------------------------------------
int vtol_topology_object::numinf(void) const
{
  return _inferiors.size();
}

//---------------------------------------------------------------------------
// -- Number of superiors
//---------------------------------------------------------------------------
int vtol_topology_object::numsup(void) const
{
  return _superiors.size();
}

//---------------------------------------------------------------------------
// -- Return the superiors list
//---------------------------------------------------------------------------
const vcl_vector<vtol_topology_object_ref> *
vtol_topology_object::superiors(void) const
{
  vcl_vector<vtol_topology_object_ref> *result;
  vcl_list<vtol_topology_object_ref>::const_iterator i;

  result=new vcl_vector<vtol_topology_object_ref>();
  result->reserve(_superiors.size());
  for(i=_superiors.begin();i!=_superiors.end();++i)
    result->push_back(*i);

  // check
  assert(_superiors.size()==result->size());

  return result;
}

//---------------------------------------------------------------------------
// -- Return the inferiors list
//---------------------------------------------------------------------------
const vcl_vector<vtol_topology_object_ref> *
vtol_topology_object::inferiors(void) const
{
  return &_inferiors;
}

//---------------------------------------------------------------------------
// Name: spatial_type
// Name: Return the spatial type
//---------------------------------------------------------------------------
vtol_topology_object::vsol_spatial_object_3d_type
vtol_topology_object::spatial_type(void) const
{
  return TOPOLOGYOBJECT;
}

//---------------------------------------------------------------------------
// -- Return the topology type
//---------------------------------------------------------------------------
vtol_topology_object::vtol_topology_object_type
vtol_topology_object::topology_type(void) const
{
  return TOPOLOGY_NO_TYPE;
}

//***************************************************************************
// Basic operations
//***************************************************************************

//---------------------------------------------------------------------------
// -- Link `this' with an inferior `inferior'
// Require: valid_inferior_type(inferior) and !is_inferior(inferior)
//---------------------------------------------------------------------------
void vtol_topology_object::link_inferior(vtol_topology_object &inferior)
{
 


  // require
  

  assert(valid_inferior_type(inferior));
  
  

  assert(!is_inferior(inferior));

  

  _inferiors.push_back(&inferior);

  

  ref();
  inferior.link_superior(*this);

  

  --ref_count;
  touch();

  
}

//---------------------------------------------------------------------------
// -- Unlink `this' with the inferior `inferior'
// Require: valid_inferior_type(inferior) and is_inferior(inferior)
//---------------------------------------------------------------------------
void vtol_topology_object::unlink_inferior(vtol_topology_object &inferior)
{
  // require
  assert(valid_inferior_type(inferior));
  assert(is_inferior(inferior));
  
  vcl_vector<vtol_topology_object_ref>::iterator i;
  
  // for(i=_inferiors.begin();(i!=_inferiors.end())&&(*(*i)!=inferior); ++i);
  for(i=_inferiors.begin();(i!=_inferiors.end())&&
	((*i).ptr()!=&inferior); ++i);
    ;
  inferior.unlink_superior(*this);
  _inferiors.erase(i);
  touch();
}

//---------------------------------------------------------------------------
// -- Unlink `this' with all its inferiors
//---------------------------------------------------------------------------
void vtol_topology_object::unlink_all_inferiors(void)
{
  while(_inferiors.size()>0)
    {
      (*_inferiors.begin())->unlink_superior(*this);
      _inferiors.erase(_inferiors.begin());
    }
  touch();
}

//---------------------------------------------------------------------------
// -- Unlink `this' of the network
//---------------------------------------------------------------------------
void vtol_topology_object::unlink(void)
{
  while(_superiors.size()>0)
    (*_superiors.begin())->unlink_inferior(*this);
  unlink_all_inferiors();
}

//***************************************************************************
// WARNING: the 2 following methods are directly called only by the superior
// class. It is FORBIDDEN to use them directly
// If you want to link and unlink superior use sup.link_inferior(*this)
// of sup.unlink_inferior(*this) 
//***************************************************************************

//---------------------------------------------------------------------------
// -- Link `this' with a superior `superior'
// Require: valid_superior_type(superior) and !is_superior(superior)
//---------------------------------------------------------------------------
void vtol_topology_object::link_superior(vtol_topology_object &superior)
{
  // require
  assert(valid_superior_type(superior));
  assert(!is_superior(superior));
  vcl_list<vtol_topology_object_ref>::iterator i;

  _superiors.push_back(&superior);
  i=_superiors.end();
  --i;
  (*i).unprotect();
  touch();
}

//---------------------------------------------------------------------------
// -- Unlink `this' with its superior `superior'
// Require: valid_superior_type(superior) and is_superior(superior)
//---------------------------------------------------------------------------
void vtol_topology_object::unlink_superior(vtol_topology_object &superior)
{


  // require
  assert(valid_superior_type(superior));
  assert(is_superior(superior));

  vcl_list<vtol_topology_object_ref>::iterator i;

  // for(i=_superiors.begin();(i!=_superiors.end())&&(*(*i)!=superior); ++i)
  for(i=_superiors.begin();(i!=_superiors.end())&&
	((*i).ptr()!=&superior); ++i)
   ;
  // check
  assert(*(*i)==superior);

  _superiors.erase(i); // unlink
  touch();


}


// -- get list of vertices 

vertex_list* vtol_topology_object::vertices(void) 
{
  vertex_list* new_list=new vertex_list;
  inf_sup_cache->vertices(*new_list);
  return new_list;
  
}

// -- get list of vertices 

void vtol_topology_object::vertices(vertex_list& verts) 
{
  inf_sup_cache->vertices(verts); 
}

// -- get list of zero_chains 
zero_chain_list* vtol_topology_object::zero_chains(void) 
{
  zero_chain_list* new_list=new zero_chain_list;
  inf_sup_cache->zero_chains(*new_list);
  return new_list;
}


// -- get list of zero chains
void vtol_topology_object::zero_chains(zero_chain_list &zerochains) 
{
  inf_sup_cache->zero_chains(zerochains);
}

// -- get list of edges

edge_list* vtol_topology_object::edges(void) 
{
  edge_list* new_list=new edge_list;
  inf_sup_cache->edges(*new_list);
  return new_list;
}

// -- get list of edges

void vtol_topology_object::edges(edge_list &edges) 
{
  inf_sup_cache->edges(edges);
}

// -- get list of one chains

one_chain_list* vtol_topology_object::one_chains(void) 
{
  one_chain_list* new_list=new one_chain_list;
  inf_sup_cache->one_chains(*new_list);
  return new_list;

}

// -- get list of one chains

void vtol_topology_object::one_chains(one_chain_list &onechains) 
{
  inf_sup_cache->one_chains(onechains);
}

// -- get list of faces

face_list *vtol_topology_object::faces(void) 
{
  face_list *new_list=new face_list;
  inf_sup_cache->faces(*new_list);
  return new_list;
}

// -- get list of faces

void vtol_topology_object::faces(face_list &face_list)
{
  inf_sup_cache->faces(face_list);
}

// -- get list of two chains 

two_chain_list *vtol_topology_object::two_chains(void) 
{
  two_chain_list *new_list=new two_chain_list;
  inf_sup_cache->two_chains(*new_list);
  return new_list;

}

// -- get list of two chains 

void vtol_topology_object::two_chains(two_chain_list &new_list) 
{
  inf_sup_cache->two_chains(new_list);
}


// -- get list of blocks

block_list *vtol_topology_object::blocks(void) 
{
  block_list *new_list=new block_list;
  inf_sup_cache->blocks(*new_list);
  return new_list;
}


// -- get list of blocks

void vtol_topology_object::blocks(block_list &new_list) 
{
  inf_sup_cache->blocks(new_list);
}


// -- print the object
void vtol_topology_object::print(vcl_ostream &strm) const
{
  strm<<"<vtol_topology_object "<<(void *)this<<">"<<vcl_endl;
  strm<<"number of inferiors "<<numinf()<<vcl_endl;
  strm<<"number of superiors "<<numsup()<<vcl_endl;
}

void vtol_topology_object::describe_inferiors(vcl_ostream &strm,
                                                 int blanking) const
{
  vcl_vector<vtol_topology_object_ref>::const_iterator i;

  for (int n=0; n<blanking; ++n) strm << ' ';
  if(_inferiors.size()==0)
    strm<<"**INFERIORS:  Empty"<<vcl_endl;
  else
    strm<<"**INFERIORS:"<<vcl_endl;
  
  for(i=_inferiors.begin();i!=_inferiors.end();++i) {
    for (int n=0; n<blanking; ++n) strm << ' ';
    (*i)->print();
  }
}

void vtol_topology_object::describe_superiors(vcl_ostream &strm,
                                                 int blanking) const
{
  vcl_list<vtol_topology_object_ref>::const_iterator i;

  for (int n=0; n<blanking; ++n) strm << ' ';
  if(_superiors.size()==0)
    strm<<"**SUPERIORS:  Empty"<<vcl_endl;
  else
    strm<<"**SUPERIORS:"<<vcl_endl;
  
 
  for(i=_superiors.begin();i!= _superiors.end();++i) {
    for (int n=0; n<blanking; ++n) strm << ' ';
    (*i)->print();
  }
}

void vtol_topology_object::describe(vcl_ostream &strm,
                                       int blanking) const
{
  describe_inferiors(strm,blanking);
  describe_superiors(strm,blanking);
}


// temperary methods used for testing


//---------------------------------------------------------------------------
// -- Compute lists of vertices 
//---------------------------------------------------------------------------
vcl_vector<vtol_vertex *> *vtol_topology_object::compute_vertices(void)
{       
  vcl_cout << "Compute vertices" << vcl_endl;
  return 0;
}


//---------------------------------------------------------------------------
// -- Compute lists of zero chains 
//---------------------------------------------------------------------------
vcl_vector<vtol_zero_chain *> *
vtol_topology_object::compute_zero_chains(void)
{       
  vcl_cout << "Compute zero_chains" << vcl_endl;
  return 0;
}

//---------------------------------------------------------------------------
// -- compute lists of edges 
//---------------------------------------------------------------------------

vcl_vector<vtol_edge *> *vtol_topology_object::compute_edges(void)
{       
  vcl_cout << "Compute edges" << vcl_endl;
  return 0;
}

//---------------------------------------------------------------------------
// -- compute lists of one chains 
//---------------------------------------------------------------------------
vcl_vector<vtol_one_chain *> *
vtol_topology_object::compute_one_chains(void)
{       
  vcl_cout << "Compute one chains" << vcl_endl;
  return 0;
}

//---------------------------------------------------------------------------
// -- compute lists of faces 
//---------------------------------------------------------------------------
vcl_vector<vtol_face *> *vtol_topology_object::compute_faces(void)
{       
  vcl_cout << "Compute faces" << vcl_endl;
  return 0;
}

//---------------------------------------------------------------------------
// -- compute lists of two chains 
//---------------------------------------------------------------------------
vcl_vector<vtol_two_chain *> *
vtol_topology_object::compute_two_chains(void)
{       
  vcl_cout << "Compute two chains" << vcl_endl;
  return 0;
}

//---------------------------------------------------------------------------
// -- compute lists of blocks 
//---------------------------------------------------------------------------
vcl_vector<vtol_block *> *vtol_topology_object::compute_blocks(void)
{       
  vcl_cout << "Compute blocks" << vcl_endl;
  return 0;
}

//#include <vcl_rel_ops.h> // gcc 2.7
//VCL_INSTANTIATE_INLINE(bool operator!=(vtol_topology_object const &, vtol_topology_object const &));
