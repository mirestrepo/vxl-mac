//*****************************************************************************
// File name: vtol_hierarchy_node_2d_ref.h
// Description: A smart pointer on a vtol_hierarchy_node_2d
//-----------------------------------------------------------------------------
// Language: C++
//
// Version |Date      | Author                   |Comment
// --------+----------+--------------------------+-----------------------------
// 1.0     |2000/05/16| Fran�ois BERTEL          |Creation
//*****************************************************************************
#ifndef VTOL_HIERARCHY_NODE_2D_REF_H
#define VTOL_HIERARCHY_NODE_2D_REF_H

class vtol_hierarchy_node_2d;

//*****************************************************************************
// External declarations for values
//*****************************************************************************
#include <vbl/vbl_smart_ptr.h>

typedef vbl_smart_ptr<vtol_hierarchy_node_2d> vtol_hierarchy_node_2d_ref;

#endif // #ifndef VTOL_HIERARCHY_NODE_2D_REF_H
