#include "dvs_global_flow/image_warped_events.h"
#include <opencv2/imgproc/imgproc.hpp>


void warpEvent(
  const cv::Point2d& vel,
  const dvs_msgs::Event& event,
  const double t_ref,
  cv::Point2d* warped_pt
)
{
  // Warp event according to flow model: displacement = velocity * time
  // FILL IN ...
  //for (const dvs_msgs::Event& event)
  //{
  double tk = event.ts.toSec(); //get time of correspond event
  double time_ = tk - t_ref;
  warped_pt->x = event.x - vel.x * time_;
  warped_pt->y = event.y - vel.y * time_;
  //}
  
}


void accumulateWarpedEvent(
  const dvs_msgs::Event& event,
  const int img_width,
  const int img_height,
  const cv::Point2d& ev_warped_pt,
  cv::Mat* image_warped,
  const OptionsWarp& optsWarp
)
{
  const float polarity = (optsWarp.use_polarity_) ? 2.f * static_cast<float>(event.polarity) - 1.f : 1.f;

  // Accumulate warped events, using bilinear voting (polarity or count)
  const int xx = ev_warped_pt.x,
            yy = ev_warped_pt.y;

  // if warped point is within the image, accumulate polarity
  if (1 <= xx && xx < img_width-2 && 1 <= yy && yy < img_height-2)
  {
    // Accumulate warped events on the IWE
    // FILL IN ...  image_warped (4 pixels)
    double sigma_x = abs(ev_warped_pt.x - xx - 0.5)/1. ;
    double sigma_y = abs(ev_warped_pt.y - yy - 0.5)/1. ;
    image_warped->at<double>(yy,xx) += polarity * (1 - sigma_y)*(1 - sigma_x);
    image_warped->at<double>(yy,xx + 1) += polarity * (1 - sigma_y)*(sigma_x);
    image_warped->at<double>(yy + 1,xx) += polarity * (sigma_y)*(1 - sigma_x);
    image_warped->at<double>(yy + 1,xx + 1) += polarity * (sigma_y)*(sigma_x);
  }
}


void computeImageOfWarpedEvents(
  const cv::Point2d& vel,
  const std::vector<dvs_msgs::Event>& events_subset,
  const cv::Size& img_size,
  cv::Mat* image_warped,
  const OptionsWarp& optsWarp
)
{
  const int width = img_size.width;
  const int height = img_size.height;

  // Create image of warped events (IWE)
  // FILL IN ...
  // hint:  *image_warped = ...
  *image_warped = cv::Mat::zeros(height, width, CV_64FC1);

  // Loop through all events
  const double t_ref = events_subset.front().ts.toSec(); // warp wrt 1st event
  
  for (const dvs_msgs::Event& ev : events_subset)
  {
    // Warp event according to candidate flow and accumulate on the IWE
    // FILL IN ...
    // hint: Call warpEvent() and accumulateWarpedEvent()
    //std::cout << "ev = " << ev;
    cv::Point2d warped_pt;
    warpEvent(vel, ev, t_ref, &warped_pt);
    accumulateWarpedEvent(ev, width, height, warped_pt, image_warped, optsWarp);  
  }

  // Smooth the IWE (to spread the votes)
  if (optsWarp.blur_sigma_ > 0)
  {
    // FILL IN ...
    // hint: cv::GaussianBlur()
    cv::GaussianBlur(*image_warped, *image_warped, cv::Size(0,0), optsWarp.blur_sigma_);
  }
}
