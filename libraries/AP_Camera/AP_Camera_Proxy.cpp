#include "AP_Camera_Proxy.h"
#include "AP_Camera_Backend.h"
#include "AP_Camera_Servo.h"
#include "AP_Camera_Relay.h"
#include "AP_Camera_SoloGimbal.h"
#include "AP_Camera_Mount.h"
#include "AP_Camera_MAVLink.h"
#include "AP_Camera_MAVLinkCamV2.h"
#include "AP_Camera_Scripting.h"
#include <GCS_MAVLink/GCS.h>

#if AP_CAMERA_PROXY_ENABLED

extern const AP_HAL::HAL& hal;

// initialize the camera proxy backend
AP_Camera_Proxy::AP_Camera_Proxy(AP_Camera &frontend, AP_Camera_Params &params, uint8_t instance) : AP_Camera_Backend(frontend, params, instance)
{
    uint8_t type = params.proxy_type.get();

    switch ((CameraType)type) {
#if AP_CAMERA_SERVO_ENABLED
    case CameraType::SERVO:
        camera_reference = new AP_Camera_Servo(frontend, params, instance);
        break;
#endif
#if AP_CAMERA_RELAY_ENABLED
    case CameraType::RELAY:
        camera_reference = new AP_Camera_Relay(frontend, params, instance);
        break;
#endif
#if AP_CAMERA_SOLOGIMBAL_ENABLED
    // check for GoPro in Solo camera
    case CameraType::SOLOGIMBAL:
        camera_reference = new AP_Camera_SoloGimbal(frontend, params, instance);
        break;
#endif
#if AP_CAMERA_MOUNT_ENABLED
    // check for Mount camera
    case CameraType::MOUNT:
        camera_reference = new AP_Camera_Mount(frontend, params, instance);
        break;
#endif
#if AP_CAMERA_MAVLINK_ENABLED
    // check for MAVLink enabled camera driver
    case CameraType::MAVLINK:
        camera_reference = new AP_Camera_MAVLink(frontend, params, instance);
        break;
#endif
#if AP_CAMERA_MAVLINKCAMV2_ENABLED
    // check for MAVLink Camv2 driver
    case CameraType::MAVLINK_CAMV2:
        camera_reference = new AP_Camera_MAVLinkCamV2(frontend, params, instance);
        break;
#endif
#if AP_CAMERA_SCRIPTING_ENABLED
    // check for Scripting driver
    case CameraType::SCRIPTING:
        camera_reference = new AP_Camera_Scripting(frontend, params, instance);
        break;
#endif
    case CameraType::NONE:
        break;
    }
}

// entry point to actually take a picture.  returns true on success
bool AP_Camera_Proxy::trigger_pic()
{
    if (camera_reference != nullptr) {
        return camera_reference->take_picture();
    }
    return false;
}

// start/stop recording video.  returns true on success
// start_recording should be true to start recording, false to stop recording
bool AP_Camera_Proxy::record_video(bool start_recording)
{
    if (camera_reference != nullptr) {
        return camera_reference->record_video(start_recording);
    }
    return false;
}

// set zoom specified as a rate or percentage
bool AP_Camera_Proxy::set_zoom(ZoomType zoom_type, float zoom_value)
{
    if (camera_reference != nullptr) {
        return camera_reference->set_zoom(zoom_type, zoom_value);
    }
    return false;
}

// set focus specified as rate, percentage or auto
// focus in = -1, focus hold = 0, focus out = 1
SetFocusResult AP_Camera_Proxy::set_focus(FocusType focus_type, float focus_value)
{
    if (camera_reference != nullptr) {
        return camera_reference->set_focus(focus_type, focus_value);
    }
    return SetFocusResult::FAILED;
}

// set tracking to none, point or rectangle (see TrackingType enum)
// if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
// p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
bool AP_Camera_Proxy::set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2)
{
    // Either MAV_CMD_CAMERA_TRACK_POINT or MAV_CMD_CAMERA_TRACK_RECTANGLE has come from the GCS
    gcs().send_text(MAV_SEVERITY_WARNING,"Proxy set tracking called");

    if (proxy_device_present == true) {

        auto _link = GCS_MAVLINK::find_by_mavtype_and_compid(MAV_TYPE_ONBOARD_CONTROLLER, proxy_device_compid, proxy_device_sysid);
        if (_link == nullptr) {
            gcs().send_text(MAV_SEVERITY_WARNING,"Tracking device not found");
            return false;
        }
        // prepare and send message
        mavlink_command_long_t pkt {};

        pkt.confirmation=0;
        pkt.target_component=proxy_device_compid;
        pkt.target_system = proxy_device_sysid;
        pkt.param1 = 0;
        pkt.param2 = 0;
        pkt.param3 = 0;
        pkt.param4 = 0;
        pkt.param5 = 0;
        pkt.param6 = 0;
        pkt.param7 = 0;

        if ((_cam_info.flags & CAMERA_CAP_FLAGS_HAS_TRACKING_POINT) && (tracking_type == TrackingType::TRK_POINT)) {
            pkt.command = MAV_CMD_CAMERA_TRACK_POINT;
            pkt.param1 = p1.x;
            pkt.param2 = p1.y;
        } else if ((_cam_info.flags & CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE) && (tracking_type == TrackingType::TRK_RECTANGLE)) {
            pkt.command = MAV_CMD_CAMERA_TRACK_RECTANGLE;
            pkt.param1 = p1.x;
            pkt.param2 = p1.y;
            pkt.param3 = p2.x;
            pkt.param4 = p2.y;
        } else if (tracking_type == TrackingType::TRK_NONE) {
            pkt.command = MAV_CMD_CAMERA_STOP_TRACKING;
        } else {
            // We don't support this type of tracking
            return false;
        }

        _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);
        gcs().send_text(MAV_SEVERITY_WARNING,"sent message to sys %d and comp %d",proxy_device_sysid,proxy_device_compid);
        return true;
    }

    if (camera_reference != nullptr) {
        return camera_reference->set_tracking(tracking_type, p1, p2);
    }
    return false;
}

// set camera lens as a value from 0 to 5
bool AP_Camera_Proxy::set_lens(uint8_t lens)
{
    if (camera_reference != nullptr) {
        return camera_reference->set_lens(lens);
    }
    return false;
}

#if AP_CAMERA_SET_CAMERA_SOURCE_ENABLED
// set_camera_source is functionally the same as set_lens except primary and secondary lenses are specified by type
bool AP_Camera_Proxy::set_camera_source(AP_Camera::CameraSource primary_source, AP_Camera::CameraSource secondary_source)
{
    if (camera_reference != nullptr) {
        return camera_reference->set_camera_source((AP_Camera::CameraSource)primary_source, (AP_Camera::CameraSource)secondary_source);
    }
    return false;
}
#endif

// send camera information message to GCS
void AP_Camera_Proxy::send_camera_information(mavlink_channel_t chan) const
{
    if (camera_reference != nullptr) {
        return camera_reference->send_camera_information(chan);
    }
}

// send camera settings message to GCS
void AP_Camera_Proxy::send_camera_settings(mavlink_channel_t chan) const
{
    if (camera_reference != nullptr) {
        return camera_reference->send_camera_settings(chan);
    }
}

// send camera capture status message to GCS
void AP_Camera_Proxy::send_camera_capture_status(mavlink_channel_t chan) const
{
    if (camera_reference != nullptr) {
        return camera_reference->send_camera_capture_status(chan);
    }
}

// handle MAVLink messages from the camera
void AP_Camera_Proxy::handle_message(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    // overrides the default CAMERA_INFORMATION handling
    if (msg.msgid == MAVLINK_MSG_ID_CAMERA_INFORMATION) {
        mavlink_msg_camera_information_decode(&msg, &_cam_info);
        proxy_device_present = true;
        proxy_device_sysid = msg.sysid;
        proxy_device_compid = msg.compid;
        const uint8_t fw_ver_major = _cam_info.firmware_version & 0x000000FF;
        const uint8_t fw_ver_minor = (_cam_info.firmware_version & 0x0000FF00) >> 8;
        const uint8_t fw_ver_revision = (_cam_info.firmware_version & 0x00FF0000) >> 16;
        const uint8_t fw_ver_build = (_cam_info.firmware_version & 0xFF000000) >> 24;

        // display camera info to user
        gcs().send_text(MAV_SEVERITY_INFO, "Camera: %s.32 %s.32 fw:%u.%u.%u.%u",
                _cam_info.vendor_name,
                _cam_info.model_name,
                (unsigned)fw_ver_major,
                (unsigned)fw_ver_minor,
                (unsigned)fw_ver_revision,
                (unsigned)fw_ver_build);
        gcs().send_text(MAV_SEVERITY_WARNING,"Sysid is %d, Compid is %d",proxy_device_sysid, proxy_device_compid);
        // _got_camera_info = true;
        return;
    }

    if (camera_reference != nullptr) {
        return camera_reference->handle_message(chan, msg);
    }
}

// configure camera
void AP_Camera_Proxy::configure(float shooting_mode, float shutter_speed, float aperture, float ISO, int32_t exposure_type, int32_t cmd_id, float engine_cutoff_time)
{
    if (camera_reference != nullptr) {
        return camera_reference->configure(shooting_mode, shutter_speed, aperture, ISO, exposure_type, cmd_id, engine_cutoff_time);
    }  
}

// handle camera control
void AP_Camera_Proxy::control(float session, float zoom_pos, float zoom_step, float focus_lock, int32_t shooting_cmd, int32_t cmd_id)
{
    if (camera_reference != nullptr) {
        return camera_reference->control(session, zoom_pos, zoom_step, focus_lock, shooting_cmd, cmd_id);
    }  
}

#if AP_CAMERA_SCRIPTING_ENABLED
// accessor to allow scripting backend to retrieve state
// returns true on success and cam_state is filled in
bool AP_Camera_Proxy::get_state(AP_Camera::camera_state_t& cam_state)
{
    if (camera_reference != nullptr) {
        return camera_reference->get_state(cam_state);
    }
    return false;
}
#endif

#endif // AP_CAMERA_PROXY_ENABLED
