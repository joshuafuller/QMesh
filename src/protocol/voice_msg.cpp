#include "voice_msg.hpp"
#include "serial_data.hpp"


auto VoiceMsgProcessor::getDataPayload() -> vector<uint8_t> {
    vector<uint8_t> pld;
    switch(radio_cb.codec2_bitrate) {
        case BITRATE_450:  return setFrames<four_frame_450bps_t>(frames); break;
        case BITRATE_700:  return setFrames<four_frame_700bps_t>(frames); break;
        case BITRATE_1200: return setFrames<four_frame_1200bps_t>(frames); break;
        case BITRATE_1300: return setFrames<four_frame_1300bps_t>(frames); break;
        case BITRATE_1400: return setFrames<four_frame_1400bps_t>(frames); break;
        case BITRATE_1600: return setFrames<four_frame_1600bps_t>(frames); break;
    }
    MBED_ASSERT(false);
}


auto VoiceMsgProcessor::getVoiceFrames(const vector<uint8_t>& pld) -> vector<vector<uint8_t>> {
    switch(radio_cb.codec2_bitrate) {
        case BITRATE_450:  return getFrames<four_frame_450bps_t>(pld, BITRATE_450); break;
        case BITRATE_700:  return getFrames<four_frame_700bps_t>(pld, BITRATE_700); break;
        case BITRATE_1200: return getFrames<four_frame_1200bps_t>(pld, BITRATE_1200); break;
        case BITRATE_1300: return getFrames<four_frame_1300bps_t>(pld, BITRATE_1300); break;
        case BITRATE_1400: return getFrames<four_frame_1400bps_t>(pld, BITRATE_1400); break;
        case BITRATE_1600: return getFrames<four_frame_1600bps_t>(pld, BITRATE_1600); break;
    }
    MBED_ASSERT(false);
}


auto VoiceMsgProcessor::addFrame(const vector<uint8_t>& frame) -> bool {
    MBED_ASSERT(frames.size() <= radio_cb.voice_frames_per_frame);
    frames.push_back(frame);
    return frames.size() == radio_cb.voice_frames_per_frame;
}