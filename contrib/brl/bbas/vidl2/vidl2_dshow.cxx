// This is brl/bbas/vidl2/vidl2_dshow_istream.cxx
//=========================================================================
//:
// \file
// \brief  DirectShow helper functions used in vidl2.
//
// See vidl2_dshow.h for details.
//
// Last modified $Date$ by $Author$.
//=========================================================================

#include <vidl2/vidl2_dshow.h>
#include <vidl2/vidl2_exception.h>
#include <vidl2/vidl2_pixel_format.h>

#include <vcl_cassert.h>
#include <vcl_iostream.h>
#include <vcl_sstream.h>
#include <vcl_map.h>
#include <vcl_utility.h>
#include <vcl_iterator.h>
#include <vcl_algorithm.h>

//-------------------------------------------------------------------------
// Helper class, which should only concern this implementation.
//-------------------------------------------------------------------------
namespace
{
  class com_manager
  {
    //SINGLETON_PROTECTION(com_manager);

  public:
    void find_capture_devices();
    void print_capture_device_names() const;
    vcl_vector<vcl_string> get_capture_device_names() const;
    CComPtr<IMoniker> get_capture_device_moniker(const vcl_string& name) const;

    //const CComPtr<IBaseFilter>& get_capture_device(int dev_num) const;

    static com_manager& instance(void)
    {
      if (!instance_) { instance_ = new com_manager; }
      return *instance_;
    }

  private:
    com_manager();
    ~com_manager();
    com_manager(const com_manager&);
    com_manager& operator=(const com_manager&);
    com_manager* operator&();

    static com_manager* instance_;

    vcl_map<vcl_string,CComPtr<IMoniker> > capture_devices_;
  };

  com_manager* com_manager::instance_ = 0;

  com_manager::com_manager(void)
  {
    // SingleThreaded Support only (for now).
#ifdef _WIN32_DCOM
    CoInitializeEx(NULL, COINIT_APARTMENTTHREADED);
#else
    CoInitialize(NULL);
#endif

    // ***** is this necessary *****
    //find_capture_devices();
  }

  com_manager::~com_manager(void)
  {
    capture_devices_.clear();
    CoUninitialize();
  }

  //: Find existing capture devices (clears the previous list, if any).
  void com_manager::find_capture_devices(void)
  {
    capture_devices_.clear();

    CComPtr<ICreateDevEnum> sys_device_enum;
    DSHOW_ERROR_IF_FAILED(
      sys_device_enum.CoCreateInstance(CLSID_SystemDeviceEnum));

    // Create an enumerator for the video capture devices.
    CComPtr<IEnumMoniker> cap_device_enum;
    HRESULT hr = sys_device_enum->CreateClassEnumerator(
      //CLSID_LegacyAmFilterCategory,
      //CLSID_VideoCompressorCategory,
      CLSID_VideoInputDeviceCategory,
      &cap_device_enum, 0);
    if (hr != S_OK) // might be S_FALSE, and FAILED(S_FALSE) != true
    {
      vcl_cerr << "Category doesn't exist or is empty.\n"
               << DSHOW_GET_ERROR_TEXT(hr) << vcl_endl;
      return;
    }

    // Iterate over the enumerated devices and add them to the map.
    CComPtr<IMoniker> moniker;
    while (cap_device_enum->Next(1, &moniker, 0) == S_OK)
    {
      CComPtr<IPropertyBag> property_bag;
      hr = moniker->BindToStorage(0, 0, IID_IPropertyBag,
                                  reinterpret_cast<void**>(&property_bag));
      if (FAILED(hr))
      {
        vcl_cerr << "BindToStorage failed for device "
                 << capture_devices_.size() + 1 << "\n"
                 << DSHOW_GET_ERROR_TEXT(hr) << vcl_endl;
        continue;
      }
      else
      {
        CComVariant var;

        if (SUCCEEDED(property_bag->Read(L"FriendlyName", &var, 0)))
        {
          assert(var.vt == VT_BSTR);

          //capture_devices_.push_back(
          //  device_wrapper(moniker, var.bstrVal, false));
          capture_devices_.insert(vcl_pair<vcl_string,CComPtr<IMoniker> >(
            vcl_string(CW2A(var.bstrVal)), moniker));
        }
        else
        {
          vcl_ostringstream oss;
          oss << capture_devices_.size();

          //capture_devices_.push_back(device_wrapper(moniker,L"N/A" + oss.str(),false));
          capture_devices_.insert(vcl_pair<vcl_string,CComPtr<IMoniker> >(
            "N/A " + oss.str(), moniker));
        }
      }

      moniker.Release(); // need to release before re-using
    }
  }

  //: Print existing capture devices.
  void com_manager::print_capture_device_names(void) const
  {
    //vcl_ostream_iterator<vcl_string> out(vcl_cout, "\n");
    vcl_vector<vcl_string> names = get_capture_device_names();

    vcl_copy(names.begin(),
             names.end(),
             vcl_ostream_iterator<vcl_string>(vcl_cout, "\n"));
  }

  //: Get existing capture devices.
  vcl_vector<vcl_string> com_manager::get_capture_device_names(void) const
  {
    vcl_vector<vcl_string> names;

    vcl_map<vcl_string,CComPtr<IMoniker> >::const_iterator iterator
      = capture_devices_.begin();

    while (iterator != capture_devices_.end())
    {
      names.push_back(iterator->first);
      iterator++;
    }

    return names;
  }

  //: Get IMoniker associated with name (call find_capture_devices first).
  CComPtr<IMoniker>
  com_manager::get_capture_device_moniker(const vcl_string& name) const
  {
    vcl_map<vcl_string,CComPtr<IMoniker> >::const_iterator iterator
      = capture_devices_.find(name);

    return iterator != capture_devices_.end() ? iterator->second : 0;
  }
}

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
namespace
{
  struct guid_string_entry
  {
    char* name;
    GUID  guid;
  };

  class guid_name_list
  {
  public:
    static vcl_string get_name(const GUID& guid);

  private:
    static guid_string_entry names[];
    static unsigned int count;
  };

  guid_string_entry guid_name_list::names[] = {
  #define OUR_GUID_ENTRY(name, l, w1, w2, b1, b2, b3, b4, b5, b6, b7, b8) \
  { #name, { l, w1, w2, { b1, b2,  b3,  b4,  b5,  b6,  b7,  b8 } } },
    #include <uuids.h>
  };
  #undef OUR_GUID_ENTRY

  unsigned int guid_name_list::count
    = sizeof(guid_name_list::names) / sizeof(guid_name_list::names[0]);

  vcl_string guid_name_list::get_name(const GUID &guid)
  {
    if (guid == GUID_NULL) { return "GUID_NULL"; }

    for (unsigned int i = 0; i < count; i++)
    {
      if (names[i].guid == guid) { return names[i].name; }
    }

    // return guids FOURCC instead.
    vcl_string fourcc;
    fourcc.push_back(static_cast<char>((guid.Data1 & 0x000000FF)      ));
    fourcc.push_back(static_cast<char>((guid.Data1 & 0x0000FF00) >>  8));
    fourcc.push_back(static_cast<char>((guid.Data1 & 0x00FF0000) >> 16));
    fourcc.push_back(static_cast<char>((guid.Data1 & 0xFF000000) >> 24));

    return fourcc;
  }
} // unnamed namespace

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//: Initialize COM (must be called before using any of these functions).
void vidl2_dshow::initialize_com(void)
{
  com_manager::instance();
}

//: Get an error description for the given HRESULT.
vcl_string
vidl2_dshow::get_error_text(const char* file, int line, HRESULT hr)
{
  TCHAR err[MAX_ERROR_TEXT_LEN];
  DWORD result = AMGetErrorText(hr, err, MAX_ERROR_TEXT_LEN);

  vcl_ostringstream oss;
  oss << file << ":" << line << ":";
  result
    ? oss << CT2A(err)
    : oss << "Unknown Error (" << vcl_hex << hr << ")";

  return oss.str();
}

//: Register a filter graph in ROT for external loading with GraphEdit.
void vidl2_dshow::register_in_rot(IUnknown* unknown, DWORD& reg)
{
  assert(unknown != 0);

  CComPtr<IRunningObjectTable> rot;
  DSHOW_ERROR_IF_FAILED(GetRunningObjectTable(0, &rot));

  vcl_wostringstream oss;
  oss << L"FilterGraph "
      << vcl_hex << reinterpret_cast<DWORD>(unknown)
      << L" pid "
      << vcl_hex << GetCurrentProcessId();
  vcl_wcout << oss.str() << vcl_endl;

  CComPtr<IMoniker> moniker;
  DSHOW_ERROR_IF_FAILED(
    CreateItemMoniker(L"!", oss.str().c_str(), &moniker));
  DSHOW_ERROR_IF_FAILED(
    rot->Register(ROTFLAGS_REGISTRATIONKEEPSALIVE, unknown, moniker,&reg));
}

//: Remove a filter graph from ROT.
void vidl2_dshow::remove_from_rot(DWORD reg)
{
  CComPtr<IRunningObjectTable> rot;
  DSHOW_ERROR_IF_FAILED(GetRunningObjectTable(0, &rot));
  DSHOW_ERROR_IF_FAILED(rot->Revoke(reg));
}

//: Save filter graph to a *.grf file.
void vidl2_dshow::save_graph_to_file(const CComPtr<IFilterGraph2>& filter_graph,
                                     const vcl_wstring& filename)
{
  assert(filter_graph != 0);

  CComPtr<IStorage> storage;
  DSHOW_ERROR_IF_FAILED(StgCreateDocfile(filename.c_str(),
                                         STGM_CREATE
                                         | STGM_TRANSACTED
                                         | STGM_READWRITE
                                         | STGM_SHARE_EXCLUSIVE,
                                         0, &storage));

  CComPtr<IStream> stream;
  //const vcl_wstring stream_name = L"ActiveMovieGraph"; 
  DSHOW_ERROR_IF_FAILED(storage->CreateStream(L"ActiveMovieGraph",
                                              STGM_CREATE
                                              | STGM_WRITE
                                              | STGM_SHARE_EXCLUSIVE,
                                              0, 0, &stream));

  CComPtr<IPersistStream> persist_stream;
  filter_graph->QueryInterface(IID_IPersistStream,
                               reinterpret_cast<void**>(&persist_stream));

  DSHOW_ERROR_IF_FAILED(persist_stream->Save(stream, TRUE));
  DSHOW_ERROR_IF_FAILED(storage->Commit(STGC_DEFAULT));
}

//: Load filter graph from a *.grf file.
void vidl2_dshow::load_graph_from_file(const CComPtr<IFilterGraph2>& filter_graph,
                                       const vcl_wstring& filename)
{
  assert(filter_graph != 0);

  CComPtr<IStorage> storage;
  if (S_OK != StgIsStorageFile(filename.c_str()))
  { // might return S_FALSE; can't use FAILED(hr)
    vidl2_exception_error(vidl2_dshow_exception(
      "Not a storage file.\n" + DSHOW_GET_ERROR_TEXT(E_FAIL)));
  }
  DSHOW_ERROR_IF_FAILED(StgOpenStorage(filename.c_str(), 0, 
                                       STGM_TRANSACTED
                                       | STGM_READ
                                       | STGM_SHARE_DENY_WRITE, 
                                       0, 0, &storage));

  CComPtr<IPersistStream> persist_stream;

  filter_graph->QueryInterface(IID_IPersistStream,
                               reinterpret_cast<void**>(&persist_stream));

  CComPtr<IStream> stream;
  DSHOW_ERROR_IF_FAILED(storage->OpenStream(L"ActiveMovieGraph", 0, 
                                            STGM_READ
                                            | STGM_SHARE_EXCLUSIVE,
                                            0, &stream));

  DSHOW_ERROR_IF_FAILED(persist_stream->Load(stream));
}

//: Get GUID name or FOURCC.
vcl_string vidl2_dshow::get_guid_name(const GUID& guid)
{
  return guid_name_list::get_name(guid);
}

//: Get multimedia subtype GUID from FOURCC.
GUID vidl2_dshow::get_guid_from_fourcc(const vcl_string& fourcc)
{
  unsigned long fourcc_cast = static_cast<unsigned long>(fourcc[0])
                            | static_cast<unsigned long>(fourcc[1] <<  8)
                            | static_cast<unsigned long>(fourcc[2] << 16)
                            | static_cast<unsigned long>(fourcc[3] << 24);

  const GUID guid = {
    fourcc_cast,
    0x0000,
    0x0010,
    { 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71 }
  };

  return guid;
}

//: Extract information from AM_MEDIA_TYPE object.
void vidl2_dshow::get_media_info(const AM_MEDIA_TYPE& amt,
                                 unsigned int& width,
                                 unsigned int& height,
                                 vidl2_pixel_format& pixel_format)
{
  // Examine the format block.
  if ( (amt.formattype == FORMAT_VideoInfo)
    && (amt.cbFormat >= sizeof(VIDEOINFOHEADER))
    && (amt.pbFormat != 0) )
  {
    VIDEOINFOHEADER *vih;
    vih = reinterpret_cast<VIDEOINFOHEADER*>(amt.pbFormat);

    width  = vih->bmiHeader.biWidth;
    height = vih->bmiHeader.biHeight < 0
           ? -vih->bmiHeader.biHeight
           :  vih->bmiHeader.biHeight;
    // ***** figure out how to handle bottom_up dibs...
    //if (vih->bmiHeader.biHeight < 0)
    //{
    //  is_bottom_up_ = true;
    //}
  }
  else 
  {
    // SampleGrabber requires VIDEOINFOHEADER type.
    //   Wrong format. Free the format block and bail out.
    DSHOW_ERROR_IF_FAILED(VFW_E_INVALIDMEDIATYPE);
  }

  if      (amt.subtype == MEDIASUBTYPE_RGB24)
  {
    pixel_format = VIDL2_PIXEL_FORMAT_BGR_24;
  }
  else if (amt.subtype == MEDIASUBTYPE_RGB555)
  {
    pixel_format = VIDL2_PIXEL_FORMAT_RGB_555;
  }
  else if (amt.subtype == MEDIASUBTYPE_RGB565)
  {
    pixel_format = VIDL2_PIXEL_FORMAT_RGB_565;
  }
  else if (amt.subtype == MEDIASUBTYPE_RGB8)
  {
    pixel_format = VIDL2_PIXEL_FORMAT_MONO_8;
  }
  //else if (amt.subtype == MEDIASUBTYPE_ARGB32)
  //{
  //  pixel_format = VIDL2_PIXEL_FORMAT_ABGR_32;
  //}
  else if (amt.subtype == MEDIASUBTYPE_YUY2)
  {
    pixel_format = VIDL2_PIXEL_FORMAT_YUYV_422;
  }
  else if (amt.subtype == MEDIASUBTYPE_UYVY)
  {
    pixel_format = VIDL2_PIXEL_FORMAT_UYVY_422;
  }
  else if (amt.subtype == MEDIASUBTYPE_YV12)
  {
    pixel_format = VIDL2_PIXEL_FORMAT_YVU_420P;
  }
  else if (amt.subtype == MEDIASUBTYPE_IYUV
        || amt.subtype == vidl2_dshow::get_guid_from_fourcc("I420"))
  {
    pixel_format = VIDL2_PIXEL_FORMAT_YUV_420P;
  }
  else if (amt.subtype == MEDIASUBTYPE_Y41P)
  {
    pixel_format = VIDL2_PIXEL_FORMAT_UYVY_411;
  }
  //else if (amt.subtype == MEDIASUBTYPE_YVU9)
  //{
  //  pixel_format = ;
  //}
  else if (amt.subtype == vidl2_dshow::get_guid_from_fourcc("DX50"))
  { // MEDIASUBTYPE_DX50
    pixel_format = VIDL2_PIXEL_FORMAT_UNKNOWN;
  }
  else if (amt.subtype == vidl2_dshow::get_guid_from_fourcc("MP4S"))
  { // WMMEDIASUBTYPE_MP4S
    pixel_format = VIDL2_PIXEL_FORMAT_UNKNOWN;
  }
  else { DSHOW_ERROR_IF_FAILED(VFW_E_INVALIDMEDIATYPE); }
}

//: Delete AM_MEDIA_TYPE memory.
void vidl2_dshow::delete_media_type(AM_MEDIA_TYPE& amt)
{
  if (amt.cbFormat != 0)
  {
    CoTaskMemFree(reinterpret_cast<void*>(amt.pbFormat));
    amt.cbFormat = 0;
    amt.pbFormat = 0;
  }
  if (amt.pUnk != 0)
  {
    // Unecessary because pUnk should not be used, but safest.
    amt.pUnk->Release();
    amt.pUnk = 0;
  }
}

//: Print a list of capture device names (i.e., FriendlyName)
void vidl2_dshow::print_capture_device_names(void)
{
  com_manager& com = com_manager::instance();

  com.find_capture_devices();

  com.print_capture_device_names();
}

//: Get a list of capture device names (i.e., FriendlyName)
vcl_vector<vcl_string> vidl2_dshow::get_capture_device_names(void)
{
  com_manager& com = com_manager::instance();

  com.find_capture_devices();

  return com.get_capture_device_names();
}

//: Get IMoniker associated with name.
CComPtr<IMoniker>
vidl2_dshow::get_capture_device_moniker(const vcl_string& name)
{
  com_manager& com = com_manager::instance();

  com.find_capture_devices();

  return com.get_capture_device_moniker(name);
}

////: 
//void
//vidl2_dshow::build_filter_graph(/*CComPtr<ICaptureGraphBuilder2>& graph_builder,*/
//                                CComPtr<IFilterGraph2>&  filter_graph,
//                                CComPtr<IMoniker>&       moniker,
//                                CComPtr<ISampleGrabber>& sample_grabber)
//{
//  assert(/*graph_builder == NULL &&*/
//    filter_graph  == NULL &&
//    moniker       != NULL);
//
//  CComPtr<ICaptureGraphBuilder2> graph_builder;
//
//  // create the filter graph manager
//  DSHOW_ERROR_IF_FAILED(filter_graph.CoCreateInstance(CLSID_FilterGraph));
//
//  // create the capture graph builder
//  DSHOW_ERROR_IF_FAILED(
//    graph_builder.CoCreateInstance(CLSID_CaptureGraphBuilder2));
//
//  // initialize the capture graph builder
//  graph_builder->SetFiltergraph(filter_graph);
//
//  // add the selected source filter to filter graph
//  CComPtr<IBaseFilter> source_filter;
//  DSHOW_ERROR_IF_FAILED(filter_graph->AddSourceFilterForMoniker(
//    moniker, 0, L"Source", &source_filter));
//
//
//  CComPtr<IAMVideoProcAmp> am_video_proc_amp;
//  long value, min, max, delta, dflt, flags, flags2;
//  source_filter->QueryInterface(IID_IAMVideoProcAmp, reinterpret_cast<void**>(&am_video_proc_amp));
//  am_video_proc_amp->Get(VideoProcAmp_Brightness, &value, &flags);
//  am_video_proc_amp->GetRange(VideoProcAmp_Brightness, &min, &max, &delta, &dflt, &flags2);
//  vcl_cout << value  << " , "
//           << min    << " , "
//           << max    << " , "
//           << delta  << " , "
//           << dflt   << " , "
//           << flags  << " , "
//           << flags2 << vcl_endl;
//
//  CComPtr<IESFProperties> esf_properties;
//
//  source_filter->QueryInterface(IID_IESFProperties, reinterpret_cast<void**>(&esf_properties));
//  esf_properties->GetBrightness(&value);
//  esf_properties->GetBrightnessRange(&min, &max, &dflt);
//  vcl_cout << value  << " , "
//           << min    << " , "
//           << max    << " , "
//           << dflt   << vcl_endl;
//  esf_properties.Release();
//
//  DSHOW_ERROR_IF_FAILED(graph_builder->FindInterface(
//    &PIN_CATEGORY_CAPTURE,//PIN_CATEGORY_PREVIEW,LOOK_DOWNSTREAM_ONLY,
//    0,//&MEDIATYPE_Video,
//    source_filter,
//    IID_IESFProperties,
//    reinterpret_cast<void**>(&esf_properties)));
//  esf_properties->GetBrightness(&value);
//  esf_properties->GetBrightnessRange(&min, &max, &dflt);
//  vcl_cout << value  << " , "
//           << min    << " , "
//           << max    << " , "
//           << dflt   << vcl_endl;
//  esf_properties.Release();
//
//  DSHOW_ERROR_IF_FAILED(graph_builder->FindInterface(
//    &PIN_CATEGORY_PREVIEW,//LOOK_DOWNSTREAM_ONLY,
//    0,//&MEDIATYPE_Video,
//    source_filter,
//    IID_IESFProperties,
//    reinterpret_cast<void**>(&esf_properties)));
//  esf_properties->GetBrightness(&value);
//  esf_properties->GetBrightnessRange(&min, &max, &dflt);
//  vcl_cout << value  << " , "
//           << min    << " , "
//           << max    << " , "
//           << dflt   << vcl_endl;
//  esf_properties.Release();
//
//
//  // *** configure the Video Output params before connecting the pin ***
//  CComPtr<IAMStreamConfig> am_stream_config;
//  DSHOW_ERROR_IF_FAILED(graph_builder->FindInterface(
//    &PIN_CATEGORY_CAPTURE,//PIN_CATEGORY_PREVIEW,LOOK_DOWNSTREAM_ONLY,
//    0,//&MEDIATYPE_Video,
//    source_filter,
//    IID_IAMStreamConfig,
//    reinterpret_cast<void**>(&am_stream_config)));
//
//  int count,size;
//  am_stream_config->GetNumberOfCapabilities(&count,&size);
//  vcl_cout << "GetNumberOfCapabilities: " << count << " , " << size << vcl_endl;
//
//  VIDEO_STREAM_CONFIG_CAPS video_stream_config_caps;
//  assert(sizeof(video_stream_config_caps) == size);
//
//  AM_MEDIA_TYPE* am_media_type = 0;
//  for (int i = 0; i < count; i++)
//  {
//    am_stream_config->GetStreamCaps(
//      i,
//      &am_media_type,
//      reinterpret_cast<BYTE*>(&video_stream_config_caps));
//
//    //vcl_cout << video_stream_config_caps.InputSize.cx << "\t"
//    //         << video_stream_config_caps.InputSize.cy << "\t"
//    //         << GuidNames[am_media_type->subtype] << vcl_endl;
//
//    if (am_media_type != NULL)
//    {
//      if (am_media_type->cbFormat != 0)
//      {
//        CoTaskMemFree(reinterpret_cast<void*>(am_media_type->pbFormat));
//        am_media_type->cbFormat = 0;
//        am_media_type->pbFormat = 0;
//      }
//      if (am_media_type->pUnk != 0)
//      {
//        // Unecessary because pUnk should not be used, but safest.
//        am_media_type->pUnk->Release();
//        am_media_type->pUnk = 0;
//      }
//
//      CoTaskMemFree(am_media_type);
//    }
//  }
//
//  // create sample grabber filter and add it to filter graph
//  CComPtr<IBaseFilter> sample_grabber_filter;
//  DSHOW_ERROR_IF_FAILED(
//    sample_grabber_filter.CoCreateInstance(CLSID_SampleGrabber));
//  DSHOW_ERROR_IF_FAILED(
//    filter_graph->AddFilter(sample_grabber_filter, L"Sample Grabber"));
//
//  //CComPtr<ISampleGrabber> sample_grabber;
//  sample_grabber_filter->QueryInterface(&sample_grabber);
//
//  AM_MEDIA_TYPE mt;
//  ZeroMemory(&mt, sizeof(AM_MEDIA_TYPE));
//  mt.majortype = MEDIATYPE_Video;
//  //mt.subtype = MEDIASUBTYPE_RGB24;
//  sample_grabber->SetMediaType(&mt);
//  sample_grabber->SetBufferSamples(true);
//  sample_grabber->SetOneShot(true);
//
//  // create a null renderer for preview pin
//  // *** is this required always? test different configurations... ***
//  CComPtr<IBaseFilter> null_renderer;
//  DSHOW_ERROR_IF_FAILED(
//    null_renderer.CoCreateInstance(CLSID_NullRenderer));
//  DSHOW_ERROR_IF_FAILED(
//    filter_graph->AddFilter(null_renderer, L"Null Renderer"));
//
//  // *** MSDN docs suggest turning the graph clock off (if not needed)
//  //     for running the graph faster. Check this.
//  // ***
//
//  // *** connect the pins ***
//  //PIN_CATEGORY_CAPTURE
//  //MEDIATYPE_Video
//  DSHOW_ERROR_IF_FAILED(graph_builder->RenderStream(0, 0,
//                                                    source_filter,
//                                                    sample_grabber_filter,
//                                                    null_renderer));
//}

void vidl2_dshow::connect_filters(CComPtr<IFilterGraph2>& filter_graph,
                                  CComPtr<IBaseFilter>& source,
                                  CComPtr<IBaseFilter>& target)
{
  //assert(!filter_graph);
  //assert(!source);
  //assert(!target);

  // All the needed pin & pin enumerator pointers
  CComPtr<IEnumPins> source_pins_enum;
  CComPtr<IEnumPins> target_pins_enum;

  CComPtr<IPin> target_pin;

  // Get the pin enumerators for both the filters
  DSHOW_ERROR_IF_FAILED(source->EnumPins(&source_pins_enum));
  DSHOW_ERROR_IF_FAILED(target->EnumPins(&target_pins_enum));

  // Loop on every pin on the source Filter
  PIN_DIRECTION pin_dir;

  CComPtr<IPin> source_pin;
  while (S_OK == source_pins_enum->Next(1, &source_pin, 0))
  {
    // Make sure that we have the output pin of the source filter
    if (FAILED(source_pin->QueryDirection(&pin_dir))
      || pin_dir == PINDIR_INPUT)
    {
      source_pin.Release();
      continue;
    }

    // I have an output pin; loop on every pin on the target Filter
    while (S_OK == target_pins_enum->Next(1, &target_pin, 0) )
    {
      if (FAILED(target_pin->QueryDirection(&pin_dir))
        || pin_dir == PINDIR_OUTPUT)
      {
        target_pin.Release();
        continue;
      }

      // Try to connect them and exit if u can, else loop more
      if(SUCCEEDED(filter_graph->ConnectDirect(source_pin, target_pin, 0)))
      {
        return;
      }

      target_pin.Release();
    }

    DSHOW_ERROR_IF_FAILED(target_pins_enum->Reset());

    source_pin.Release();
  }

  vidl2_exception_error(vidl2_dshow_exception(
    "Couldn't connect the filters." + DSHOW_GET_ERROR_TEXT(E_FAIL)));
}

