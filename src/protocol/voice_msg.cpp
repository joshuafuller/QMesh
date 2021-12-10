#include "serial_data.hpp"
#include "voice_msg.hpp"


auto VoiceMsgProcessor::getDataPayload() -> vector<uint8_t> {
    PORTABLE_ASSERT(radio_cb.valid);
    PORTABLE_ASSERT(radio_cb.net_cfg.voice_frames_per_frame == 2 ||
                radio_cb.net_cfg.voice_frames_per_frame == 4);
    if(radio_cb.net_cfg.voice_frames_per_frame == 2) {
        switch(radio_cb.net_cfg.codec2_bitrate) {
            case BITRATE_450:  return setFrames<two_frame_450bps_t>(frames); break;
            case BITRATE_700:  return setFrames<two_frame_700bps_t>(frames); break;
            case BITRATE_1200: return setFrames<two_frame_1200bps_t>(frames); break;
            case BITRATE_1300: return setFrames<two_frame_1300bps_t>(frames); break;
            case BITRATE_1400: return setFrames<two_frame_1400bps_t>(frames); break;
            case BITRATE_1600: return setFrames<two_frame_1600bps_t>(frames); break;
            case BITRATE_2400: return setFramesBig<two_frame_2400bps_t>(frames); break;
            case BITRATE_3200: return setFramesBig<two_frame_3200bps_t>(frames); break;
        }
        PORTABLE_ASSERT(false);
    } else if(radio_cb.net_cfg.voice_frames_per_frame == 4) {
        switch(radio_cb.net_cfg.codec2_bitrate) {
            case BITRATE_450:  return setFrames<four_frame_450bps_t>(frames); break;
            case BITRATE_700:  return setFrames<four_frame_700bps_t>(frames); break;
            case BITRATE_1200: return setFrames<four_frame_1200bps_t>(frames); break;
            case BITRATE_1300: return setFrames<four_frame_1300bps_t>(frames); break;
            case BITRATE_1400: return setFrames<four_frame_1400bps_t>(frames); break;
            case BITRATE_1600: return setFrames<four_frame_1600bps_t>(frames); break;
            case BITRATE_2400: return setFramesBig<four_frame_2400bps_t>(frames); break;
            case BITRATE_3200: return setFramesBig<four_frame_3200bps_t>(frames); break;
        }
        PORTABLE_ASSERT(false);
    } else {
        PORTABLE_ASSERT(false);
    }
}


auto VoiceMsgProcessor::size() -> int {
    PORTABLE_ASSERT(radio_cb.valid);
    PORTABLE_ASSERT(radio_cb.net_cfg.voice_frames_per_frame == 2 ||
                radio_cb.net_cfg.voice_frames_per_frame == 4);
    if(radio_cb.net_cfg.voice_frames_per_frame == 2) {
        switch(radio_cb.net_cfg.codec2_bitrate) {
            case BITRATE_450:  return sizeof(two_frame_size_450bps_t); break;
            case BITRATE_700:  return sizeof(two_frame_size_700bps_t); break;
            case BITRATE_1200: return sizeof(two_frame_size_1200bps_t); break;
            case BITRATE_1300: return sizeof(two_frame_size_1300bps_t); break;
            case BITRATE_1400: return sizeof(two_frame_size_1400bps_t); break;
            case BITRATE_1600: return sizeof(two_frame_size_1600bps_t); break;
            case BITRATE_2400: return sizeof(two_frame_size_2400bps_t); break;
            case BITRATE_3200: return sizeof(two_frame_size_3200bps_t); break;
        }
        PORTABLE_ASSERT(false);
    } else if(radio_cb.net_cfg.voice_frames_per_frame == 4) {
        switch(radio_cb.net_cfg.codec2_bitrate) {
            case BITRATE_450:  return sizeof(four_frame_450bps_t); break;
            case BITRATE_700:  return sizeof(four_frame_700bps_t); break;
            case BITRATE_1200: return sizeof(four_frame_1200bps_t); break;
            case BITRATE_1300: return sizeof(four_frame_1300bps_t); break;
            case BITRATE_1400: return sizeof(four_frame_1400bps_t); break;
            case BITRATE_1600: return sizeof(four_frame_1600bps_t); break;
            case BITRATE_2400: return sizeof(four_frame_2400bps_t); break;
            case BITRATE_3200: return sizeof(four_frame_3200bps_t); break;
        }
        PORTABLE_ASSERT(false);
    } else {
        PORTABLE_ASSERT(false);
    }
}


auto VoiceMsgProcessor::getVoiceFrames(const vector<uint8_t>& pld) -> vector<vector<uint8_t>> {
    PORTABLE_ASSERT(radio_cb.valid);
    PORTABLE_ASSERT(radio_cb.net_cfg.voice_frames_per_frame == 2 ||
                radio_cb.net_cfg.voice_frames_per_frame == 4);
    if(radio_cb.net_cfg.voice_frames_per_frame == 2) {
        switch(radio_cb.net_cfg.codec2_bitrate) {
            case BITRATE_450:  return getFrames<two_frame_450bps_t>(pld, BITRATE_450); break;
            case BITRATE_700:  return getFrames<two_frame_700bps_t>(pld, BITRATE_700); break;
            case BITRATE_1200: return getFrames<two_frame_1200bps_t>(pld, BITRATE_1200); break;
            case BITRATE_1300: return getFrames<two_frame_1300bps_t>(pld, BITRATE_1300); break;
            case BITRATE_1400: return getFrames<two_frame_1400bps_t>(pld, BITRATE_1400); break;
            case BITRATE_1600: return getFrames<two_frame_1600bps_t>(pld, BITRATE_1600); break;
            case BITRATE_2400: return getFramesBig<two_frame_2400bps_t>(pld, BITRATE_2400); break;
            case BITRATE_3200: return getFramesBig<two_frame_3200bps_t>(pld, BITRATE_3200); break;
        }
        PORTABLE_ASSERT(false);
    } else if(radio_cb.net_cfg.voice_frames_per_frame == 4) {
        switch(radio_cb.net_cfg.codec2_bitrate) {
            case BITRATE_450:  return getFrames<four_frame_450bps_t>(pld, BITRATE_450); break;
            case BITRATE_700:  return getFrames<four_frame_700bps_t>(pld, BITRATE_700); break;
            case BITRATE_1200: return getFrames<four_frame_1200bps_t>(pld, BITRATE_1200); break;
            case BITRATE_1300: return getFrames<four_frame_1300bps_t>(pld, BITRATE_1300); break;
            case BITRATE_1400: return getFrames<four_frame_1400bps_t>(pld, BITRATE_1400); break;
            case BITRATE_1600: return getFrames<four_frame_1600bps_t>(pld, BITRATE_1600); break;
            case BITRATE_2400: return getFramesBig<four_frame_2400bps_t>(pld, BITRATE_2400); break;
            case BITRATE_3200: return getFramesBig<four_frame_3200bps_t>(pld, BITRATE_3200); break;
        } 
        PORTABLE_ASSERT(false);
    } else {
        PORTABLE_ASSERT(false);
    }
}


auto VoiceMsgProcessor::addFrame(const vector<uint8_t>& frame) -> bool {
    PORTABLE_ASSERT(radio_cb.valid);
    PORTABLE_ASSERT(frames.size() <= radio_cb.net_cfg.voice_frames_per_frame);
    frames.push_back(frame);
    return frames.size() == radio_cb.net_cfg.voice_frames_per_frame;
}