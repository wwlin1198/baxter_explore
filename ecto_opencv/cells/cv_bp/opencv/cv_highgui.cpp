#include <boost/python.hpp>
#include <boost/python/args.hpp>
#include <boost/python/overloads.hpp>
#include <boost/python/dict.hpp>
#include <boost/python/tuple.hpp>

#include <opencv2/highgui/highgui.hpp>
#if CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION == 4 && CV_SUBMINOR_VERSION < 10
#include <cv_backports/imshow.hpp>
#else
namespace cv_backports {
  using cv::destroyWindow;
  using cv::imshow;
  using cv::namedWindow;
  using cv::setMouseCallback;
  using cv::setWindowProperty;
  using cv::startWindowThread;
  using cv::waitKey;
}
#endif

#include <fstream>
#include <iostream>
#include <map>

namespace bp = boost::python;

namespace
{
  BOOST_PYTHON_FUNCTION_OVERLOADS(imread_overloads,cv::imread,1,2)
  ;

  BOOST_PYTHON_FUNCTION_OVERLOADS(imwrite_overloads,cv::imwrite,2,3)
  ;
  BOOST_PYTHON_FUNCTION_OVERLOADS(imencode_overloads,cv::imencode,3,4)
  ;
#if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >= 3)
  BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(videowriter_open_overloads1,cv::VideoWriter::open,4,5);
  BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(videowriter_open_overloads2,cv::VideoWriter::open,5,6);
#endif

  struct PyMCallBackData
  {
    bp::object cb,udata;

    static void callback_fn(int event,int x, int y, int flags, void* param)
    {
      PyMCallBackData* d = static_cast<PyMCallBackData*>(param);
      d->cb(event,x,y,flags,d->udata);
    }
    static std::map<std::string,PyMCallBackData*> callbacks_;
  };
  
  std::map<std::string,PyMCallBackData*> PyMCallBackData::callbacks_;
  //typedef void (*MouseCallback )(int event, int x, int y, int flags, void* param);
  //CV_EXPORTS void setMouseCallback( const string& windowName, MouseCallback onMouse, void* param=0)
  void setMouseCallback_(const std::string& windowName, bp::object callback, bp::object userdata)
  {
    if(callback == bp::object())
    {
        PyMCallBackData::callbacks_[windowName] = NULL;
        cv_backports::setMouseCallback(windowName,NULL,NULL);
        return;
    }
    //FIXME get rid of this leak...
    PyMCallBackData* d = new PyMCallBackData;
    d->cb = callback;
    d->udata = userdata;
    PyMCallBackData::callbacks_[windowName] = d;
    cv_backports::setMouseCallback(windowName,&PyMCallBackData::callback_fn,d);
  }
}
namespace opencv_wrappers
{
  void wrap_highgui_defines();
  void wrap_video_capture()
  {
    bp::class_<cv::VideoCapture> VideoCapture_("VideoCapture");
    VideoCapture_.def(bp::init<>());
    VideoCapture_.def(bp::init<std::string>());
    VideoCapture_.def(bp::init<int>());
#if CV_MAJOR_VERSION == 3
    typedef bool(cv::VideoCapture::*open_1)(const cv::String&);
    typedef bool(cv::VideoCapture::*open_2)(int);
#else
    typedef bool(cv::VideoCapture::*open_1)(const std::string&);
    typedef bool(cv::VideoCapture::*open_2)(int);
#endif
    VideoCapture_.def("open", open_1(&cv::VideoCapture::open));
    VideoCapture_.def("open", open_2(&cv::VideoCapture::open));
    VideoCapture_.def("isOpened", &cv::VideoCapture::isOpened);
    VideoCapture_.def("release", &cv::VideoCapture::release);
    VideoCapture_.def("grab", &cv::VideoCapture::grab);
    VideoCapture_.def("retrieve", &cv::VideoCapture::retrieve);
    VideoCapture_.def("read", &cv::VideoCapture::read);
    VideoCapture_.def("set", &cv::VideoCapture::set);
    VideoCapture_.def("get", &cv::VideoCapture::get);
  }

  void wrap_video_writer()
  {
    bp::class_<cv::VideoWriter> VideoWriter_("VideoWriter");
    VideoWriter_.def(bp::init<>());
    VideoWriter_.def(bp::init<const std::string&, int, double, cv::Size, bool>());
#if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >= 3)
    VideoWriter_.def("open",
      static_cast<bool(cv::VideoWriter::*)(const cv::String&,int,double,cv::Size,bool)>(
      &cv::VideoWriter::open),videowriter_open_overloads1());
    VideoWriter_.def("open",
      static_cast<bool(cv::VideoWriter::*)(const cv::String&,int,int,double,cv::Size,bool)>(
      &cv::VideoWriter::open),videowriter_open_overloads2());
#else
    VideoWriter_.def("open", &cv::VideoWriter::open);
#endif
    VideoWriter_.def("isOpened", &cv::VideoWriter::isOpened);
    VideoWriter_.def("write", &cv::VideoWriter::write);
  }

  int waitKey(int millis)
  {
    return 255 & cv_backports::waitKey(millis);
  }

  void wrap_highgui()
  {
    wrap_highgui_defines();
    //video stuff.
    wrap_video_capture();
    wrap_video_writer();

    //image windows
#if CV_MAJOR_VERSION == 3
    bp::def("imshow", static_cast<void (*)(const cv::String&, cv::InputArray)>(cv::imshow));
#else
    bp::def("imshow", cv_backports::imshow);
#endif
    bp::def("waitKey", waitKey);
    bp::def("namedWindow", cv_backports::namedWindow);
//CV_EXPORTS void setMouseCallback( const string& windowName, MouseCallback onMouse, void* param=0);
    bp::def("setMouseCallback", setMouseCallback_);
    //image io
    bp::def("imread", cv::imread, imread_overloads());
    bp::def("imwrite", cv::imwrite, imwrite_overloads());
    cv::Mat (*f1)(cv::InputArray, int) = &cv::imdecode;
    bp::def("imdecode", f1);
#if (CV_MAJOR_VERSION > 2) || ((CV_MAJOR_VERSION==2) && ((CV_MINOR_VERSION>4) || (CV_MINOR_VERSION==4 && CV_SUBMINOR_VERSION>=3)))
    cv::Mat (*f2)(cv::InputArray, int, cv::Mat*) = &cv::imdecode;
    bp::def("imdecode", f2);
#endif
    bp::def("imencode", cv::imencode, imencode_overloads());
  }
}
