#ifndef gmvl_database_h_
#define gmvl_database_h_
#ifdef __GNUC__
#pragma interface
#endif
/*
  crossge@crd.ge.com
*/

// this provides a (not necessary fast or efficient) multi-access database.
//   only dependency is on 'tag' nodes, although tag nodes are in all other
//   sense identical to other nodes.

#include <vcl_iostream.h>
#include <vcl_vector.h>
#include <vcl_string.h>

#include <gmvl/gmvl_node_cache.h>
#include <gmvl/gmvl_connection_cache.h>

class gmvl_database
{
public:

  // methods on database nodes
  void add_node( const gmvl_node_sptr node);
  void remove_node( const gmvl_node_sptr node);

  // methods on connections
  void add_connection( const gmvl_node_sptr node1, const gmvl_node_sptr node2);
  void add_connections( const gmvl_node_sptr node1, vcl_vector<gmvl_node_sptr> nodes);

  // clever accessors

  //   nodes of a particular type
  vcl_vector<gmvl_node_sptr> get_nodes( const vcl_string type) const;

  //   nodes with a particular connection list
  vcl_vector<gmvl_node_sptr> get_connected_nodes( const gmvl_node_sptr node) const;
  vcl_vector<gmvl_node_sptr> get_connected_nodes( const gmvl_node_sptr node1, const gmvl_node_sptr node2) const;
  vcl_vector<gmvl_node_sptr> get_connected_nodes( const gmvl_node_sptr node1, const gmvl_node_sptr node2, const gmvl_node_sptr node3) const;
  vcl_vector<gmvl_node_sptr> get_connected_nodes( const vcl_vector<gmvl_node_sptr> nodes) const;

  //   nodes with a particular connection list and a particular type
  vcl_vector<gmvl_node_sptr> get_connected_nodes( const gmvl_node_sptr node, const vcl_string type) const;
  vcl_vector<gmvl_node_sptr> get_connected_nodes( const vcl_vector<gmvl_node_sptr> nodes, const vcl_string type) const;

  // lookup a particular tag (returns null if not found)
  gmvl_node_sptr find_tag( const vcl_string &string) const;

  // output
  friend vcl_ostream &operator<<( vcl_ostream &os, const gmvl_database &db);

protected:

  gmvl_node_cache nodecache_;
  gmvl_connection_cache connectioncache_;
};

vcl_ostream &operator<<( vcl_ostream &os, const gmvl_database &db);


#endif
