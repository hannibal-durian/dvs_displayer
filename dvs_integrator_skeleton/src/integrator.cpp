#include "dvs_integrator/integrator.h"
#include <cv_bridge/cv_bridge.h>
#include <math.h>

namespace dvs_integrator {

Integrator::Integrator(ros::NodeHandle & nh, ros::NodeHandle nh_private) : nh_(nh)
{
  // Get parameters of the method
  nh_private.param<double>("cut_off", alpha_cutoff_, 5.);

  // Set up subscribers and publishers
  // FILL IN...
//  event_sub_ = ...
  event_sub_ = nh_.subscribe("events", 0, &Integrator::eventsCallback, this);
  // Set the queue size to infinity to avoid dropping messages.

  image_transport::ImageTransport it_(nh_);
  // FILL IN...
//  time_map_pub_ = ...
//  image_pub_ = ...
  time_map_pub_ = it_.advertise("time_map", 1);
  image_pub_ = it_.advertise("image_out", 1);

  // Dynamic reconfigure
  dynamic_reconfigure_callback_ = boost::bind(&Integrator::reconfigureCallback, this, _1, _2);
  server_.reset(new dynamic_reconfigure::Server<dvs_integrator::dvs_integratorConfig>(nh_private));
  server_->setCallback(dynamic_reconfigure_callback_);

  // Contrast thresholds
  // Later we may use varying contrast sensitivities
  c_pos_ = 0.1;
  c_neg_ = 0.1;
  t_last_=0.;
}


Integrator::~Integrator()
{
  // Close the publishers
  // FILL IN...
  time_map_pub_.shutdown();
  image_pub_.shutdown();
}


/**
 * Process input event messages
 */
void Integrator::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // Need to update the state (time_map and brightness image) even if there are no subscribers on the output image

  std::cout << "alpha_cutoff_ = " << alpha_cutoff_ << std::endl;
  
  const double t_first_ = msg->events[0].ts.toSec();
  //const double t_last_ = msg->events[-1].ts.toSec();
  const bool non_monotonic_time = (t_first_<t_last_);
  
  if (!(state_time_map_.rows == msg->height && state_time_map_.cols == msg->width) || non_monotonic_time)
  {//for check of the images(input)
    // Allocate memory for time map and for image out
    
    //const double t_first_ = msg->events[0].ts.toSec();
    // Initialize the time map with the time of the first event
    // and initialize the brightness image to zero.
    // FILL IN...
//    state_time_map_ = cv::Mat ...
//    state_image_    = cv::Mat ...
    state_time_map_ = cv::Mat(msg->height, msg->width, CV_64FC1, cv::Scalar(t_first_));
    state_image_    = cv::Mat::zeros(msg->height, msg->width, CV_64FC1);
  }
  
  //const int events_per_frame = 2e4;
  //int count_event = 0;
  // Process events in the message msg, one by one (in a loop)
  // FILL IN...
  for (const dvs_msgs::Event& ev : msg->events)
  {
	double tk = ev.ts.toSec();
	double p;
	//count_event +=1;
	if (ev.polarity)
	{
		p = 1.;
		p = p*c_pos_;
	}
	else if (!ev.polarity)
	{
	 	p = -1.;
	 	p = p*c_neg_;
	}
	//p =(ev.polarity ? 1. : -1.);
	//const double row = ev.y;
	//const double height = ev.x;
	double beta = exp(-alpha_cutoff_*(tk-state_time_map_.at<double>(ev.y,ev.x)));
	state_image_.at<double>(ev.y,ev.x) = beta*state_image_.at<double>(ev.y,ev.x)+p; 
	state_time_map_.at<double>(ev.y,ev.x) = tk;
	
	t_last_ = tk;
	/*if (count_event % events_per_frame ==0)
	//if (bool sensor_msgs::Image& image : msg->Image)
	{
		for (int i =0; i<state_time_map_.rows;i++ )
		{
			for(int j =0; j<state_time_map_.cols;j++)
			{
			beta = exp(-alpha_cutoff_*(tk-state_time_map_.at<double>(i,j)));
			state_image_.at<double>(i,j) = beta*state_image_.at<double>(i,j);
			}
		}
		//beta = exp(-alpha_cutoff_ * (tk - state_time_map_));
		//state_image_ *= beta;
		state_time_map_ = cv::Mat(msg->height, msg->width, CV_64FC1, cv::Scalar(tk));
	}*/
  }


  // Exit if there are no subscribers
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

  // Need to display all pixels at the same reference time (e.g., the time of
  // the last event). So do not forget to "decay" the brightness to the ref. time.
  // FILL IN...
  
  cv::Mat cv_state_copy = state_image_.clone();
  double alpha;
  for (int i =0; i<state_image_.rows;i++ )
	{
		for(int j =0; j<state_image_.cols;j++)
		{
			alpha = exp(-alpha_cutoff_*(t_last_-state_time_map_.at<double>(i,j)));
			cv_state_copy.at<double>(i,j) = alpha*state_image_.at<double>(i,j);
		}
	}


  // Convert to appropriate range, [0,255]
  // Feel free to use minMaxLocRobust
  // FILL IN...
 //cv::Mat image_;
  double rmin_val_image, rmax_val_image;
  minMaxLocRobust(cv_state_copy, rmin_val_image, rmax_val_image, 5.);
  const double scale_image = ((rmax_val_image != rmin_val_image) ? 255. / (rmax_val_image - rmin_val_image) : 1.);
  cv::Mat cv_image_normalized = scale_image * (cv_state_copy - rmin_val_image);
  cv_image_normalized.convertTo(cv_image.image, CV_8UC1);
  
  //cv::Mat state_time_map_;
  double rmin_val_time, rmax_val_time;
  minMaxLocRobust(state_time_map_, rmin_val_time, rmax_val_time,5.);
  const double scale_time = ((rmax_val_time != rmin_val_time) ? 255. / (rmax_val_time - rmin_val_time) : 1.);
  cv::Mat cv_image_time_normalized = scale_time * (state_time_map_ - rmin_val_time);
  cv_image_time_normalized.convertTo(cv_image_time.image, CV_8UC1);



  // Publish the time map and the brightness image
  time_map_pub_.publish(cv_image_time.toImageMsg());
  image_pub_.publish(cv_image.toImageMsg());
}


/**
 * Interface with the parameters that can be changed online via dynamic reconfigure
 */
void Integrator::reconfigureCallback(dvs_integrator::dvs_integratorConfig &config, uint32_t level)
{
  alpha_cutoff_ = config.Cutoff_frequency;
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

} // namespace
