// ZhangCameraNode.h: interface for the ZhangCamera class.
//
//////////////////////////////////////////////////////////////////////

#ifndef AFX_ZHANGCAMERA_H__EB787129_58FA_4195_A386_71D7CC0C9546__INCLUDED_
#define AFX_ZHANGCAMERA_H__EB787129_58FA_4195_A386_71D7CC0C9546__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#include "CameraNode.h"
#include <vcl_vector.h>
#include <vgl/vgl_homg_point_2d.h>

class ZhangCameraNode : public CameraNode
{
 private:
   vcl_vector< vgl_homg_point_2d<double> > *pPointLists_; 
   int nViews_;
 public:
   vcl_vector< vgl_homg_point_2d<double> >& getPoints(int iView) { return pPointLists_[iView];}
   int readData(char* fname, int iView);  // for debugging
   ZhangCameraNode(int id, int nViews);
   virtual ~ZhangCameraNode();
};

#endif // AFX_ZHANGCAMERA_H__EB787129_58FA_4195_A386_71D7CC0C9546__INCLUDED_
