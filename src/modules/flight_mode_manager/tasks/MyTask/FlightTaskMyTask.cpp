#include "FlightTaskMyTask.hpp"
#include <uORB/uORB.h>
#include <uORB/topics/point_line_img.h>

struct Vec3{
    float v[3];
};

bool FlightTaskMyTask::activate(const trajectory_setpoint_s &last_setpoint)
{
  bool ret = FlightTask::activate(last_setpoint);
  return ret;
}

bool FlightTaskMyTask::update()
{
   
  

  
  /* subscribe to vehicle_acceleration topic */
  int point_line_img_sub_fd = orb_subscribe(ORB_ID(point_line_img));

  /* one could wait for multiple topics with this technique, just using one here */
  px4_pollfd_struct_t fds[] = {
	{ .fd = point_line_img_sub_fd,   .events = POLLIN }
  };

  int error_counter = 0;

  for (int i = 0; i < 10; i++) {
        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
        int poll_ret = px4_poll(fds, 1, 1000);

        /* handle the poll result */
        if (poll_ret == 0) {
                /* this means none of our providers is giving us data */
                PX4_ERR("Got no data within a second");

        } else if (poll_ret < 0) {
                /* this is seriously bad - should be an emergency */
                if (error_counter < 10 || error_counter % 50 == 0) {
                        /* use a counter to prevent flooding (and slowing us down) */
                        PX4_ERR("ERROR return value from poll(): %d", poll_ret);
                }

                error_counter++;
        } else {
                if (fds[0].revents & POLLIN) {
                        /* obtained data for the first file descriptor */
                        struct Vec3 raw;
                        /* copy sensors raw data into local buffer */
                        orb_copy(ORB_ID(point_line_img), point_line_img_sub_fd, &raw);
                        struct Vec3 received_point_line_img1;
                        for (int j=0; j<2; j++)
                              received_point_line_img1.v[j] = raw.v[j];
                        PX4_INFO("PT1 = (%8.4f\t%8.4f)", (double)received_point_line_img1.v[1], (double)received_point_line_img1.v[2]);
                }
        }
  }
  return true;
}
