// ZhangCamera.cpp: implementation of the ZhangCamera class.
//
//////////////////////////////////////////////////////////////////////

#include "ZhangCamera.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

ZhangCameraNode::ZhangCameraNode()
{
	// build lens distortion model
	vcl_vector<bool> flags(7, false);
	flags[0] = true;
	flags[1] = true;
	_pCam -> setLensModel(flags);

	// allocate space to store features.
	_pImageLists = new vcl_list<HomoPoint2D> [n]
}

ZhangCameraNode::~ZhangCameraNode()
{

}
