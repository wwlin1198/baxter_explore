#include <opencv2/opencv.hpp>
#include "poseRT.hpp"
//#include "utils.hpp"

using namespace cv;
using std::cout;
using std::endl;

void createProjectiveMatrix(const cv::Mat &R, const cv::Mat &t, cv::Mat &Rt)
{
  CV_Assert(R.type() == CV_64FC1);
  CV_Assert(t.type() == CV_64FC1);

  Rt.create(4, 4, CV_64FC1);
  Rt.at<double>(3, 0) = 0.;
  Rt.at<double>(3, 1) = 0.;
  Rt.at<double>(3, 2) = 0.;
  Rt.at<double>(3, 3) = 1.;

  Mat roi_R = Rt(Range(0, 3), Range(0, 3));
  CV_Assert(roi_R.rows == 3 && roi_R.cols == 3);

  if(R.rows == 3 && R.cols == 3)
  {
    R.copyTo(roi_R);
  }
  else
  {
    Mat fullR;
    Rodrigues(R, fullR);
    fullR.copyTo(roi_R);
  }

  Mat roi_t = Rt(Range(0, 3), Range(3,4));
  t.copyTo(roi_t);
}

void getRvecTvec(const Mat &projectiveMatrix, Mat &rvec, Mat &tvec)
{
  CV_Assert(projectiveMatrix.rows == 4 && projectiveMatrix.cols == 4);

  Rodrigues(projectiveMatrix(Range(0, 3), Range(0, 3)), rvec);
  projectiveMatrix(Range(0, 3), Range(3, 4)).copyTo(tvec);

  CV_Assert(rvec.rows == 3 && rvec.cols == 1);
  CV_Assert(tvec.rows == 3 && tvec.cols == 1);
  CV_Assert(rvec.type() == CV_64FC1 && tvec.type() == CV_64FC1);
}

PoseRT::PoseRT()
{
  dim = 3;
  rvec = Mat::zeros(dim, 1, CV_64FC1);
  tvec = Mat::zeros(dim, 1, CV_64FC1);
}

PoseRT::PoseRT(const cv::Mat &projectiveMatrix)
{
  dim = 3;
  setProjectiveMatrix(projectiveMatrix);
}

PoseRT::PoseRT(const cv::Mat &rotation, const cv::Mat &translation)
{
  CV_Assert(rotation.type() == CV_64FC1);
  CV_Assert(translation.type() == CV_64FC1);

  dim = 3;

  CV_Assert(translation.rows == dim && translation.cols == 1);
  tvec = translation.clone();

  if (rotation.rows == dim && rotation.cols == 1)
  {
    rvec = rotation.clone();
  }
  else
  {
    if (rotation.rows == dim && rotation.cols == dim)
    {
      Rodrigues(rotation, rvec);
    }
    else
    {
      CV_Assert(false);
    }
  }
}

PoseRT::PoseRT(const PoseRT &pose)
{
  rvec = pose.rvec.clone();
  tvec = pose.tvec.clone();
}

PoseRT& PoseRT::operator=(const PoseRT &pose)
{
  if (this != &pose)
  {
    rvec = pose.rvec.clone();
    tvec = pose.tvec.clone();
  }
  return (*this);
}

cv::Mat PoseRT::getRvec() const
{
  return rvec;
}

cv::Mat PoseRT::getTvec() const
{
  return tvec;
}

cv::Mat PoseRT::getRotationMatrix() const
{
  Mat rotation;
  Rodrigues(rvec, rotation);
  return rotation;
}

cv::Mat PoseRT::getProjectiveMatrix() const
{
  Mat Rt;
  createProjectiveMatrix(rvec, tvec, Rt);
  return Rt;
}

cv::Mat PoseRT::getQuaternion() const
{
  Mat quaternion(4, 1, CV_64FC1, Scalar(0));
  double angle = norm(getRvec());
  CV_Assert(getRvec().type() == CV_64FC1);
  const double eps = 1e-5;
  if (angle > eps)
  {
    quaternion.at<double>(0) = getRvec().at<double>(0) * sin(angle / 2.0) / angle;
    quaternion.at<double>(1) = getRvec().at<double>(1) * sin(angle / 2.0) / angle;
    quaternion.at<double>(2) = getRvec().at<double>(2) * sin(angle / 2.0) / angle;
    quaternion.at<double>(3) = cos(angle / 2.0);
  }
  else
  {
    quaternion.at<double>(3) = 1.0;
  }
  return quaternion;
}

void PoseRT::setRotation(const cv::Mat &rotation)
{
  CV_Assert(rotation.rows == 3 && rotation.cols == 3);
  CV_Assert(rotation.type() == CV_64FC1);

  Rodrigues(rotation, rvec);
}

void PoseRT::setProjectiveMatrix(const cv::Mat &rt)
{
  if (rt.empty())
  {
    rvec = Mat::zeros(dim, 1, CV_64FC1);
    tvec = Mat::zeros(dim, 1, CV_64FC1);
  }
  else
  {
    getRvecTvec(rt, rvec, tvec);
  }
}

PoseRT PoseRT::operator*(const PoseRT &pose) const
{
  PoseRT result;
  composeRT(pose.getRvec(), pose.getTvec(), rvec, tvec, result.rvec, result.tvec);
  //composeRT(rvec, tvec, pose.getRvec(), pose.getTvec(), result.rvec, result.tvec);
  return result;
}

void PoseRT::computeDistance(const PoseRT &pose1, const PoseRT &pose2, double &rotationDistance, double &translationDistance, const cv::Mat &Rt_obj2cam)
{
  Mat Rt_diff_cam = pose1.getProjectiveMatrix() * pose2.getProjectiveMatrix().inv(DECOMP_SVD);
  Mat Rt_diff_obj = Rt_diff_cam;
  if (!Rt_obj2cam.empty())
  {
    Rt_diff_obj = Rt_obj2cam.inv(DECOMP_SVD) * Rt_diff_cam * Rt_obj2cam;
  }

  Mat rvec_diff_obj, tvec_diff_obj;
  getRvecTvec(Rt_diff_obj, rvec_diff_obj, tvec_diff_obj);
  rotationDistance = norm(rvec_diff_obj);
  translationDistance = norm(tvec_diff_obj);
}

void PoseRT::computeObjectDistance(const PoseRT &pose1, const PoseRT &pose2, double &rotationDistance, double &translationDistance)
{
  PoseRT diff_cam = pose1.inv() * pose2;

  rotationDistance = norm(diff_cam.getRvec());
  translationDistance = norm(diff_cam.getTvec());
}

double getDice()
{
  return double(rand()) / RAND_MAX;
}

PoseRT PoseRT::generateRandomPose(double rotationAngleInRadians, double translation)
{
  double phi = 2*CV_PI*getDice();
  double theta = CV_PI * getDice();

  PoseRT pose;
  pose.rvec.at<double>(0) = sin(theta) * cos(phi);
  pose.rvec.at<double>(1) = sin(theta) * sin(phi);
  pose.rvec.at<double>(2) = cos(theta);
  pose.rvec *= rotationAngleInRadians;

  pose.tvec.create(3, 1, CV_64FC1);
  phi = 2*CV_PI*getDice();
  theta = CV_PI * getDice();
  pose.tvec.at<double>(0) = sin(theta) * cos(phi);
  pose.tvec.at<double>(1) = sin(theta) * sin(phi);
  pose.tvec.at<double>(2) = cos(theta);
  pose.tvec *= translation;

  return pose;
}

void PoseRT::computeMeanPose(const std::vector<PoseRT> &poses, PoseRT &meanPose)
{
  meanPose = PoseRT();
  if (poses.empty())
  {
    return;
  }

  Mat meanTvec = meanPose.tvec;
  Mat meanRotationMatrix = meanPose.getRotationMatrix();
  for (size_t i = 0; i < poses.size(); ++i)
  {
    meanTvec += poses[i].tvec;
    meanRotationMatrix += poses[i].getRotationMatrix();
  }
  meanTvec /= poses.size();
  meanRotationMatrix /= poses.size();

  SVD svd;
  Mat w, u, vt;
  svd.compute(meanRotationMatrix, w, u, vt, SVD::FULL_UV);
  Mat meanOrthogonalRotationMatrix = u * vt;

  meanPose.tvec = meanTvec;
  meanPose.setRotation(meanOrthogonalRotationMatrix);
}

PoseRT PoseRT::obj2cam(const cv::Mat &Rt_obj2cam)
{
  Mat projectiveMatrix_obj = getProjectiveMatrix();
  Mat projectiveMatrix_cam = Rt_obj2cam * projectiveMatrix_obj * Rt_obj2cam.inv(DECOMP_SVD);

  PoseRT pose_cam(projectiveMatrix_cam);
  return pose_cam;
}

PoseRT PoseRT::inv() const
{
  Mat projectiveMatrix = getProjectiveMatrix();
  Mat invertedProjectiveMatrix = projectiveMatrix.inv(DECOMP_SVD);
  PoseRT invertedPose(invertedProjectiveMatrix);
  return invertedPose;
}

void PoseRT::write(const std::string &filename) const
{
  FileStorage fs(filename, FileStorage::WRITE);
  CV_Assert(fs.isOpened());
  write(fs);
  fs.release();
}

void PoseRT::write(cv::FileStorage &fs) const
{
  fs << "rvec" << rvec;
  fs << "tvec" << tvec;
}

void PoseRT::read(const std::string &filename)
{
  FileStorage fs(filename, FileStorage::READ);
  if (!fs.isOpened())
  {
    CV_Error(CV_StsBadArg, "cannot open the file " + filename);
  }
  read(fs.root());
  fs.release();
}

void PoseRT::read(const cv::FileNode &node)
{
  node["rvec"] >> rvec;
  node["tvec"] >> tvec;
  CV_Assert(!rvec.empty() && !tvec.empty());
}

std::ostream& operator<<(std::ostream& output, const PoseRT& pose)
{
  output << pose.rvec << " " << pose.tvec;
  return output;
}
