/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  Camera driver for cameras included in Proxy
 */
#pragma once

#include "AP_Camera_Backend.h"

#if AP_CAMERA_PROXY_ENABLED

#include "AP_Camera.h"

class AP_Camera_Proxy : public AP_Camera_Backend
{
public:

    // Constructor
    AP_Camera_Proxy(AP_Camera &frontend, AP_Camera_Params &_params, uint8_t instance);

    // using AP_Camera_Backend::AP_Camera_Backend;
    /* Do not allow copies */
    CLASS_NO_COPY(AP_Camera_Proxy);

    // initialize the camera proxy backend
    void init(AP_Camera &frontend, AP_Camera_Params &_params, uint8_t instance);

    // entry point to actually take a picture.  returns true on success
    bool trigger_pic() override;

    // start or stop video recording.  returns true on success
    // set start_recording = true to start record, false to stop recording
    bool record_video(bool start_recording) override;

    // set focus specified as rate, percentage or auto
    // focus in = -1, focus hold = 0, focus out = 1
    SetFocusResult set_focus(FocusType focus_type, float focus_value) override;

    // set tracking to none, point or rectangle (see TrackingType enum)
    // if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
    // p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
    bool set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2) override;

    // set camera lens as a value from 0 to 5
    bool set_lens(uint8_t lens) override;

    // set zoom specified as a rate or percentage
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

#if AP_CAMERA_SET_CAMERA_SOURCE_ENABLED
    // set_camera_source is functionally the same as set_lens except primary and secondary lenses are specified by type
    bool set_camera_source(AP_Camera::CameraSource primary_source, AP_Camera::CameraSource secondary_source) override;
#endif

    // handle MAVLink messages from the camera
    void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg) override;

    // configure camera
    void configure(float shooting_mode, float shutter_speed, float aperture, float ISO, int32_t exposure_type, int32_t cmd_id, float engine_cutoff_time) override;

    // handle camera control
    void control(float session, float zoom_pos, float zoom_step, float focus_lock, int32_t shooting_cmd, int32_t cmd_id) override;

    // send camera settings message to GCS
    void send_camera_settings(mavlink_channel_t chan) const override;

    // send camera capture status message to GCS
    void send_camera_capture_status(mavlink_channel_t chan) const override;

#if AP_CAMERA_SCRIPTING_ENABLED
    // accessor to allow scripting backend to retrieve state
    // returns true on success and cam_state is filled in
    bool get_state(AP_Camera::camera_state_t& cam_state) override;
#endif

    // send camera information message to GCS
    void send_camera_information(mavlink_channel_t chan) const override;

    // enum for camera type
    enum class CameraType {
        NONE = 0,           // None
#if AP_CAMERA_SERVO_ENABLED
        SERVO = 1,          // Servo/PWM controlled camera
#endif
#if AP_CAMERA_RELAY_ENABLED
        RELAY = 2,          // Relay controlled camera
#endif
#if AP_CAMERA_SOLOGIMBAL_ENABLED
        SOLOGIMBAL = 3,     // GoPro in Solo gimbal
#endif
#if AP_CAMERA_MOUNT_ENABLED
        MOUNT = 4,          // Mount library implements camera
#endif
#if AP_CAMERA_MAVLINK_ENABLED
        MAVLINK = 5,        // MAVLink enabled camera
#endif
#if AP_CAMERA_MAVLINKCAMV2_ENABLED
        MAVLINK_CAMV2 = 6,  // MAVLink camera v2
#endif
#if AP_CAMERA_SCRIPTING_ENABLED
        SCRIPTING = 7,  // Scripting backend
#endif
    };

private:
    AP_Camera_Backend* camera_reference = nullptr;
    mavlink_camera_information_t _cam_info {}; // latest camera information received from camera
    bool proxy_device_present = false;
    uint8_t proxy_device_sysid;
    uint8_t proxy_device_compid;
};

#endif // AP_CAMERA_PROXY_ENABLED
