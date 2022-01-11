#include "dvs_integrator_conv/integrator.h"
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <cassert>

namespace dvs_integrator_conv {

Integrator::Integrator(ros::NodeHandle & nh, ros::NodeHandle nh_private) : nh_(nh)
{
  // Get parameters of the method from the launch file
  nh_private.param<double>("cut_off", alpha_cutoff_, 5.);

  // Set up subscribers and publishers
  event_sub_ = nh_.subscribe("events", 0, &Integrator::eventsCallback, this);
  // set queue_size to 0 to avoid discarding messages (for correctness).

  image_transport::ImageTransport it_(nh_);
  time_map_pub_ = it_.advertise("time_map", 1);
  image_pub_ = it_.advertise("image_out", 1);

  // Dynamic reconfigure
  dynamic_reconfigure_callback_ = boost::bind(&Integrator::reconfigureCallback, this, _1, _2);
  server_.reset(new dynamic_reconfigure::Server<dvs_integrator_conv::dvs_integrator_convConfig>(nh_private));
  server_->setCallback(dynamic_reconfigure_callback_);

  // Contrast thresholds
  // Later we may use varying contrast sensitivities
  c_pos_ = 0.1;
  c_neg_ = 0.1;
  t_last_ = 0.;
}


Integrator::~Integrator()
{
  time_map_pub_.shutdown();
  image_pub_.shutdown();
}


/**
 * Process input event messages
 */
void Integrator::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // Need to update the state (time_map and brightness image) even if there are no subscribers on the output image
  const double t_first_ = msg->events[0].ts.toSec();
  const bool non_monotonic_time = (t_first_ < t_last_);
  if (!(state_time_map_.rows == msg->height && state_time_map_.cols == msg->width)
    || non_monotonic_time )
  {
    // Allocate memory for time map and for brightness image
    state_time_map_ = cv::Mat(msg->height, msg->width, CV_64FC1, cv::Scalar(t_first_));
    state_image_    = cv::Mat::zeros(msg->height, msg->width, CV_64FC1);
  }

  // Choose a kernel (outside the event loop)
  cv::Mat kernel;
  setKernel(kernel);
  int ksize = kernel.rows; // =... FILL IN...
  int hksize = (ksize-1)/2; // =... FILL IN... half kernel size: 0 if 1x1 kernel, 1 if 3x3 kenel, 2 if 5x5 kernel, etc.
  //cv::flip(kernel, kernel, -1);

  for (const dvs_msgs::Event& ev : msg->events)
  {
    const double tk = ev.ts.toSec();
    
      double p; //polarity
      double p_c; //polarity * threshold
      if (ev.polarity)
      {
	p = 1.;
	p_c = p*c_pos_;
      }
      else if (!ev.polarity)
      {
	 p = -1.;
	 p_c = p*c_neg_;
      }

    // Update state
    if ( (hksize <= ev.x) && (ev.x <= msg->width-1-hksize)
      && (hksize <= ev.y) && (ev.y <= msg->height-1-hksize) )
    {
      // Border due to convolution mask
      //cv::Mat state_image_border;
      //cv::copyMakeBorder(state_image_, state_image_border, hksize, hksize, hksize, hksize, cv::BORDER_CONSTANT, 0);

      // Current set of state pixels to be updated
      // cv::Rect ... FILL IN...
      //cv::Rect rect(ev.y-hksize, ev.x-hksize, ksize, ksize);

      // Update state: time map and map of convolved accumulated polarities
      // Implement decay on selected pixels
      // ... FILL IN...
	
      //double beta = exp(-alpha_cutoff_*(tk-state_time_map_.at<double>(ev.y,ev.x)));
      //state_image_.at<double>(ev.y,ev.x) = beta*state_image_.at<double>(ev.y,ev.x)+p_c; 
      
      for (int kernel_y = 0; kernel_y < ksize; kernel_y++) //apply kernel on neighbor pixels
      {
		for (int kernel_x = 0; kernel_x < ksize; kernel_x++)
		{
		double weight = kernel.at<double>(kernel_y, kernel_x);
		
		double beta = exp(-alpha_cutoff_*(tk-state_time_map_.at<double>(ev.y+kernel_y-hksize,ev.x+kernel_x-hksize)));
		//double beta = exp(-alpha_cutoff_*(tk-state_time_map_.at<double>(ev.y,ev.x)));
		double state_value = beta*state_image_.at<double>(ev.y+kernel_y-hksize,ev.x+kernel_x-hksize) + weight * p_c;
		
		state_image_.at<double>(ev.y+kernel_y-hksize,ev.x+kernel_x-hksize) = state_value;
		state_time_map_.at<double>(ev.y+kernel_y-hksize,ev.x+kernel_x-hksize) = tk;
		
		}
	}

	// Update time image using tk
	// ... FILL IN...
	//state_time_map_.at<double>(ev.y,ev.x) = tk;

    }
    else
    {
      // Out of bounds. Just update the time map
      //double beta = exp(-alpha_cutoff_*(tk-state_time_map_.at<double>(ev.y,ev.x)));
      //state_image_.at<double>(ev.y,ev.x) = beta*state_image_.at<double>(ev.y,ev.x)+p_c;
      state_time_map_.at<double>(ev.y,ev.x) = tk;

    }
    //t_last_ = tk;
  }
  //t_last_ = ... FILL IN...
  t_last_ = msg->events[ msg->events.size()-1 ].ts.toSec();

  

  // Publish if there are any subscribers
  if (image_pub_.getNumSubscribers() + time_map_pub_.getNumSubscribers() > 0)
  {
    publishState();
  }
}


/**
 * Publish the time map and the brightness image at latest time (Output)
 */
void Integrator::publishState()
{
  // Publish the current state (time map and image)

  // Initialize, including header
  cv_bridge::CvImage cv_image, cv_image_time;
  cv_image.header.stamp = ros::Time::now();
  cv_image.encoding = "mono8";
  cv_image_time.header.stamp = ros::Time::now();
  cv_image_time.encoding = "mono8";

  // Fill content in appropriate range [0,255]
  // Need to decay the image to the present time
  cv::Mat image_out;
  // FILL IN...
  image_out = state_image_.clone();
  double alpha;
  for (int i =0; i<image_out.rows;i++ )
	{
		for(int j =0; j<image_out.cols;j++)
		{
			alpha = exp(-alpha_cutoff_*(t_last_-state_time_map_.at<double>(i,j)));
			image_out.at<double>(i,j) = alpha*image_out.at<double>(i,j);
		}
	}

  // Map from current range (or better, from robust min and max) to [0,255]
  normalize(state_time_map_, cv_image_time.image, 5.);
  normalize(image_out, cv_image.image, 5.);

  // Publish the time map and the brightness image
  time_map_pub_.publish(cv_image_time.toImageMsg());
  image_pub_.publish(cv_image.toImageMsg());
}


void Integrator::setKernel(cv::Mat& ker)
{
  switch(convolution_mask_)
  {
    case 1:
      sobel_x_.copyTo(ker);
      break;
    // FILL IN...
    case 0:
      original_.copyTo(ker);
      break;
    case 2:
    	sobel_y_.copyTo(ker);
    	break;
    case 3:
    	laplace_.copyTo(ker);
    	break;
    case 4:
    	blur_gauss_.copyTo(ker);
    	break;
  }
}


/**
 * Interface with the parameters that can be changed online via dynamic reconfigure
 */
void Integrator::reconfigureCallback(dvs_integrator_conv::dvs_integrator_convConfig &config, uint32_t level)
{
  alpha_cutoff_ = config.Cutoff_frequency;
  convolution_mask_ = config.Convolution_mask;
}


/**
 * Compute the robust min and max pixel values of an image using percentiles,
 * e.g., 2th and 98th percentiles instead of min and max, respectively.
 *
 * Sort the pixel values of an image and discard a percentage of them,
 * from the top and the bottom, as potential outliers to compute
 * a robust version of the min and max values.
 */
void Integrator::minMaxLocRobust(const cv::Mat& image, double& rmin, double& rmax,
                                 const double& percentage_pixels_to_discard)
{
  cv::Mat image_as_row = image.reshape(0,1);
  cv::Mat image_as_row_sorted;
  cv::sort(image_as_row, image_as_row_sorted, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
  image_as_row_sorted.convertTo(image_as_row_sorted, CV_64FC1);
  const int single_row_idx_min = (0.5*percentage_pixels_to_discard/100.)*image.total();
  const int single_row_idx_max = (1 - 0.5*percentage_pixels_to_discard/100.)*image.total();
  rmin = image_as_row_sorted.at<double>(single_row_idx_min);
  rmax = image_as_row_sorted.at<double>(single_row_idx_max);
}


/**
 * Normalize src to the range [0,255] using robust min and max values
 */
void Integrator::normalize(const cv::Mat& src, cv::Mat& dst, const double& percentage_pixels_to_discard)
{
  double rmin_val, rmax_val;
  minMaxLocRobust(src, rmin_val, rmax_val, percentage_pixels_to_discard);
  const double scale = ((rmax_val != rmin_val) ? 255. / (rmax_val - rmin_val) : 1.);
  cv::Mat state_image_normalized = scale * (src - rmin_val);
  state_image_normalized.convertTo(dst, CV_8UC1);
}

} // namespace
