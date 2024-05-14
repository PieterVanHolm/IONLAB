/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file fake_linedetect.c
 * Simiulates the CV-based line detector sensor
 *
 * @author Leandro Lustosa <leandro.lustosa@isae-supaero.fr>
 */

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <poll.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/point_line_img.h>

#include <math.h>

__EXPORT int fake_linedetect_main(int argc, char *argv[]);

struct Mat3x3{
   float d[3][3];
};

struct Vec3{
    float v[3];
};

struct Quat{
    float v[4];
};

int matrix_times_vec(struct Vec3 *v_out, struct Mat3x3 *D, struct Vec3 *v_in);
int euler2dcm(float y, float p, float r, struct Mat3x3 *D);
int quat2dcm(struct Quat *q, struct Mat3x3 *D);
int matrix_times_matrix(struct Mat3x3 *R, struct Mat3x3 *A, struct Mat3x3 *B);

int fake_linedetect_main(int argc, char *argv[])
{
  //PX4_INFO("Hello Sky!");

  // malloc sensor output data (two data points (0,y,z) in the image, which portrays the line!!)
  struct Vec3 point_line_img1;
  struct Vec3 point_line_img2;
  for (int i=0; i<3; i++) point_line_img1.v[i] = 0.0f;
  for (int i=0; i<3; i++) point_line_img2.v[i] = 0.0f;

  // malloc problem constant parameters (mostly extrinsic camera params)
  struct Vec3 POS_CAM_DRO_DRO; // position of cam wrt drone in drone coordinates [meters]
  struct Mat3x3 D_CAM_DRO; // rotation from drone to camera frame

  // malloc problem variables
  struct Vec3 pos_dro_ned_ned; // position of drone wrt NED in NED coordinates [meters]
  struct Mat3x3 D_DRO_NED; // rotation from NED to drone frame
  struct Mat3x3 D_CAM_NED; // rotation from NED to camera frame

  // ground lines definition
  struct Vec3 P0_LINE;
  struct Vec3 L_LINE;

  // init problem constant parameters (mostly extrinsic camera params)
  POS_CAM_DRO_DRO.v[0] = 0.0f;
  POS_CAM_DRO_DRO.v[1] = 0.0f;
  POS_CAM_DRO_DRO.v[2] = 0.0f;
  euler2dcm(0.0f, -1.570795f, 0.0f, &D_CAM_DRO); // (PI/2) rotation
  for (int i=0; i<3; i++) P0_LINE.v[i] = 0.0f;
  for (int i=0; i<3; i++) L_LINE.v[i]  = 0.0f;
  L_LINE.v[0] = 10.0f; // this means 100m of line direction North (for example only, not IMAV)
  float FOCAL_LENGTH = 100.0f;
  float WIDTH_PIXELS = 1240.0f;
  float HEIGHT_PIXELS = 920.0f;

  int pos_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position_groundtruth));
  int att_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude_groundtruth      ));

  px4_pollfd_struct_t fds[] = {
    { .fd = pos_sub_fd,   .events = POLLIN },
    { .fd = att_sub_fd,   .events = POLLIN }
  };

  while (true) {
    /* wait for sensor update of 2 file descriptors for 1000 ms (1 second) */
    px4_poll(fds, 2, 1000);

    if (fds[0].revents & POLLIN) {
      /* obtained data for the first file descriptor (position ned) */
      struct vehicle_local_position_s raw;
      /* copy data into local buffer */
      orb_copy(ORB_ID(vehicle_local_position_groundtruth), pos_sub_fd, &raw);
      // copy data into Vec3 struct (position of drone wrt NED in NED coordinates [meters])
      pos_dro_ned_ned.v[0] = raw.x;
      pos_dro_ned_ned.v[1] = raw.y;
      pos_dro_ned_ned.v[2] = raw.z;
      // plot data for fun
      //PX4_INFO("Position:\t%8.4f\t%8.4f\t%8.4f",
      //  (double)raw.x,
      //  (double)raw.y,
      //  (double)raw.z);
    }

    if (fds[1].revents & POLLIN) {
      /* obtained data for the second file descriptor (attitude) */
      struct vehicle_attitude_s raw;
      /* copy data into local buffer */
      orb_copy(ORB_ID(vehicle_attitude_groundtruth), att_sub_fd, &raw);
      // copy data into quaternion struct
      struct Quat qu;
      for (int i = 0; i<4; i++)
        qu.v[i] = raw.q[i];
      quat2dcm(&qu, &D_DRO_NED); // rotation from NED to drone frame
      // plot data for fun
      //PX4_INFO("Attitude:\t%8.4f\t%8.4f\t%8.4f",
      //  (double)raw.q[0],
      //  (double)raw.q[1],
	    //  (double)raw.q[2]);
    }

    /** compute sensor data **/
    // for now we use exhaustion to look for beginning and ending of points in image
    float m_min = 2.0f; // clearly out of range, will be updated
    float m_max = -1.0f; // clearly out of range, will be upadated
    float m = 0.0f;
    float DM = 0.01f; // check every 10cm if that point is inside the image

    while (m<1.0f) {

      struct Vec3 pos_tar_cam_cam;
      struct Vec3 accumulator_vec3;
      // init vectors
      for (int i = 0; i<3; i++) pos_tar_cam_cam.v[i] = 0.0f;
      for (int i = 0; i<3; i++) accumulator_vec3.v[i] = 0.0f;

      matrix_times_vec(&accumulator_vec3, &D_CAM_DRO, &POS_CAM_DRO_DRO); // accumulator_vec3 = D_CAM_DRO*POS_CAM_DRO_DRO
      for (int i = 0; i<3; i++) pos_tar_cam_cam.v[i] += -accumulator_vec3.v[i]; //why add already ? can't you just use accumulator_vec3 ?

      matrix_times_matrix(&D_CAM_NED, &D_CAM_DRO, &D_DRO_NED);
      matrix_times_vec(&accumulator_vec3, &D_CAM_NED, &pos_dro_ned_ned);
      for (int i = 0; i<3; i++) pos_tar_cam_cam.v[i] += -accumulator_vec3.v[i];

      //PX4_INFO("PT1 = (%8.4f\t%8.4f\t%8.4f)", (double)pos_tar_cam_cam.v[0], (double)pos_tar_cam_cam.v[1], (double)pos_tar_cam_cam.v[2]);

      for (int i = 0; i<3; i++) accumulator_vec3.v[i] = m*L_LINE.v[i];
      for (int i = 0; i<3; i++) accumulator_vec3.v[i] += P0_LINE.v[i];
      struct Vec3 accumulator_vec3_2;
      matrix_times_vec(&accumulator_vec3_2, &D_CAM_NED, &accumulator_vec3);

      for (int i = 0; i<3; i++) pos_tar_cam_cam.v[i] += accumulator_vec3_2.v[i];

      for (int i = 2; i>=0; i--) pos_tar_cam_cam.v[i] = FOCAL_LENGTH*pos_tar_cam_cam.v[i]/pos_tar_cam_cam.v[0];

      //PX4_INFO("PT1 = (%8.4f\t%8.4f\t%8.4f)", (double)pos_tar_cam_cam.v[0], (double)pos_tar_cam_cam.v[1], (double)pos_tar_cam_cam.v[2]);

      // check if point is inside the image
      if ( -WIDTH_PIXELS/2.0f < pos_tar_cam_cam.v[1] && pos_tar_cam_cam.v[1] < WIDTH_PIXELS/2.0f ) {
        if ( -HEIGHT_PIXELS/2.0f < pos_tar_cam_cam.v[2] && pos_tar_cam_cam.v[2] < HEIGHT_PIXELS/2.0f ) {
          if ( m < m_min ) {
            m_min = m;
            for (int i=0; i<3; i++) point_line_img1.v[i] = pos_tar_cam_cam.v[i];
          }
          if ( m > m_max ) {
            m_max = m;
            for (int i=0; i<3; i++) point_line_img2.v[i] = pos_tar_cam_cam.v[i];
          }
        }
      }

      // check next point DM units away from the actual one
      m += DM;

    }

    // print line in terminal
    //PX4_INFO("PT1 = (%8.4f\t%8.4f) PT2 = (%8.4f\t%8.4f)", (double)point_line_img1.v[1], (double)point_line_img1.v[2], (double)point_line_img2.v[1], (double)point_line_img2.v[2]);

    //PX4_INFO("----------------------------------------------------------------------------------------------------");
    float p3y, p3z;
    float dly, dlz, dpy, dpz, dl;
    char print_line[100];
    for (int i=0; i<40; i++) {
      for (int j=0; j<100; j++) {
        p3y = -WIDTH_PIXELS/2.0f + (WIDTH_PIXELS/100.0f)*j;
        p3z = -HEIGHT_PIXELS/2.0f + (HEIGHT_PIXELS/40.0f)*i;
        dpy = p3y - point_line_img1.v[1];
        dpz = p3z - point_line_img1.v[2];
        dly = point_line_img2.v[1] - point_line_img1.v[1];
        dlz = point_line_img2.v[2] - point_line_img1.v[2];
        dl = sqrtf( dly*dly + dlz*dlz );
        if ( -20.0f*dl < dly*dpz - dpy*dlz && dly*dpz - dpy*dlz < 20.0f*dl ) {
          print_line[j] = 'x';
        } else {
          print_line[j] = ' ';
        }
      }
      //PX4_INFO("%s", print_line);
    }
    //PX4_INFO("----------------------------------------------------------------------------------------------------");

    // publish data regarding sensor
    orb_advert_t point_line_img_pub_fd = orb_advertise(ORB_ID(point_line_img), &point_line_img1); // advert the topic to obtain a publisher handle
    orb_publish(ORB_ID(point_line_img), point_line_img_pub_fd, &point_line_img1);
  }

  return OK;
}

int matrix_times_vec(struct Vec3 *v_out, struct Mat3x3 *D, struct Vec3 *v_in) {

  for (int i=0; i<3; i++) v_out->v[i] = 0.0f;

  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      v_out->v[i] += D->d[i][j]*v_in->v[j];

  return 0;

}

int euler2dcm(float y, float p, float r, struct Mat3x3 *D) {

  D->d[0][0] = cosf(p)*cosf(y);
  D->d[0][1] = cosf(p)*sinf(y);
  D->d[0][2] = -sinf(p);

  D->d[1][0] = -cosf(r)*sinf(y) + sinf(r)*sinf(p)*cosf(y);
  D->d[1][1] = cosf(r)*cosf(y) + sinf(r)*sinf(p)*sinf(y);
  D->d[1][2] = sinf(r)*cosf(p);

  D->d[2][0] = sinf(r)*sinf(y) + cosf(r)*sinf(p)*cosf(y);
  D->d[2][1] = -sinf(r)*cosf(y) + cosf(r)*sinf(p)*sinf(y);
  D->d[2][2] = cosf(r)*cosf(p);

  return 0;

}

int quat2dcm(struct Quat *q, struct Mat3x3 *D) {

  D->d[0][0] = q->v[0]*q->v[0] + q->v[1]*q->v[1] - q->v[2]*q->v[2] - q->v[3]*q->v[3];
  D->d[0][1] = 2.0f * ( q->v[1]*q->v[2] + q->v[0]*q->v[3] );
  D->d[0][2] = 2.0f * ( q->v[1]*q->v[3] - q->v[0]*q->v[2] );
  D->d[1][0] = 2.0f * ( q->v[1]*q->v[2] - q->v[0]*q->v[3] );
  D->d[1][1] = q->v[0]*q->v[0] - q->v[1]*q->v[1] + q->v[2]*q->v[2] - q->v[3]*q->v[3];
  D->d[1][2] = 2.0f * ( q->v[2]*q->v[3] + q->v[0]*q->v[1] );
  D->d[2][0] = 2.0f * ( q->v[1]*q->v[3] + q->v[0]*q->v[2] );
  D->d[2][1] = 2.0f * ( q->v[2]*q->v[3] - q->v[0]*q->v[1] );
  D->d[2][2] = q->v[0]*q->v[0] - q->v[1]*q->v[1] - q->v[2]*q->v[2] + q->v[3]*q->v[3];

  return 0;

}

int matrix_times_matrix(struct Mat3x3 *R, struct Mat3x3 *A, struct Mat3x3 *B) {

  struct Vec3 colR;
  struct Vec3 colB;

  for (int j=0; j<3; j++) {
    for (int i=0; i<3; i++) colB.v[i] = B->d[i][j];
    matrix_times_vec(&colR, A, &colB);
    for (int i=0; i<3; i++) R->d[i][j] = colR.v[i];
  }

  return 0;

}
