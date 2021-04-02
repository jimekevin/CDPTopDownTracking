#ifndef TOPDOWNTRACKING_THRESHOLDFILTER_H
#define TOPDOWNTRACKING_THRESHOLDFILTER_H
/*
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_sensor.hpp>
#include <librealsense2/hpp/rs_processing.hpp>

typedef enum rs2_custom_option {
    RS2_CUSTOM_OPTION_START = RS2_OPTION_COUNT,
    RS2_CUSTOM_OPTION_MIN_Z,
    RS2_CUSTOM_OPTION_MAX_Z,
};

// See rs_processing.hpp
class ThresholdFilter : public rs2::filter {
public:
  ThresholdFilter(float min_dist = 0.15f, float max_dist = 4.f)
          //: filter(init(), 1)
  {
      //set_option(RS2_OPTION_MIN_DISTANCE, min_dist);
      //set_option(RS2_OPTION_MAX_DISTANCE, max_dist);
      set_option(static_cast<rs2_option>(RS2_CUSTOM_OPTION_MIN_Z), min_dist);
      set_option(static_cast<rs2_option>(RS2_CUSTOM_OPTION_MAX_Z), max_dist);
  }

  rs2::frame process(rs2::frame frame) const override
  {
      invoke(frame);
      rs2::frame f;
      if (!_queue.poll_for_frame(&f))
          throw std::runtime_error("Error occured during execution of the processing block! See the log for more info");
      return f;
  }

//  ThresholdFilter(filter f) : filter(f)
//  {
//      rs2_error* e = nullptr;
//      if (!rs2_is_processing_block_extendable_to(f.get(), RS2_EXTENSION_THRESHOLD_FILTER, &e) && !e)
//      {
//          _block.reset();
//      }
//      rs2::error::handle(e);
//  }

//protected:
//  ThresholdFilter(std::shared_ptr<rs2_processing_block> block) : filter(block, 1) {}

private:
//  std::shared_ptr<rs2_processing_block> init()
//  {
//      rs2_error* e = nullptr;
//      auto block = std::shared_ptr<rs2_processing_block>(
//              rs2_create_threshold(&e),
//              rs2_delete_processing_block);
//      rs2::error::handle(e);
//
//      return block;
//  }
};
*/
#endif //TOPDOWNTRACKING_THRESHOLDFILTER_H
