#ifndef VOICE_MSG_HPP
#define VOICE_MSG_HPP


#include "mbed.h"
#include <deque>
#include <vector>


static constexpr int BITRATE_450 = 450;
static constexpr int BITRATE_700 = 700;
static constexpr int BITRATE_1200 = 1200;
static constexpr int BITRATE_1300 = 1300;
static constexpr int BITRATE_1400 = 1400;
static constexpr int BITRATE_1600 = 1600;
static vector<int> bitrates = {BITRATE_450, BITRATE_700, BITRATE_1200,
                                BITRATE_1300, BITRATE_1400, BITRATE_1600};

static constexpr int FRAME_RATE_HZ = 25;


#define FOUR_FRAME_TYPE_SMALL(NAME, BITRATE) \
using NAME = struct __attribute__((__packed__)) { \
    uint64_t num_frames : 2;  \
    uint64_t frame0     : ((BITRATE)/25); \
    uint64_t frame1     : ((BITRATE)/25); \
    uint64_t frame2     : ((BITRATE)/25); \
    uint64_t frame3     : ((BITRATE)/25); \
};

#define FOUR_FRAME_TYPE_BIG(NAME, BITRATE) \
using NAME = struct __attribute__((__packed__)) { \
    uint64_t num_frames : 2;  \
    uint64_t frame0a     : ((BITRATE)/25/2); \
    uint64_t frame0b     : ((BITRATE)/25/2); \
    uint64_t frame1a     : ((BITRATE)/25/2); \
    uint64_t frame1b     : ((BITRATE)/25/2); \
    uint64_t frame2a     : ((BITRATE)/25/2); \
    uint64_t frame2b     : ((BITRATE)/25/2); \
    uint64_t frame3a     : ((BITRATE)/25/2); \
    uint64_t frame3b     : ((BITRATE)/25/2); \
};

FOUR_FRAME_TYPE_SMALL(four_frame_450bps_t, 450)
FOUR_FRAME_TYPE_SMALL(four_frame_700bps_t, 700)
FOUR_FRAME_TYPE_SMALL(four_frame_1200bps_t, 1200)
FOUR_FRAME_TYPE_SMALL(four_frame_1300bps_t, 1300)
FOUR_FRAME_TYPE_SMALL(four_frame_1400bps_t, 1400)
FOUR_FRAME_TYPE_SMALL(four_frame_1600bps_t, 1600)
FOUR_FRAME_TYPE_BIG(four_frame_2400bps_t, 2400)
FOUR_FRAME_TYPE_BIG(four_frame_3200bps_t, 3200)

template<class T> 
auto setFrames(vector<vector<uint8_t>> frames) -> vector<uint8_t> {
    T frame{};
    frame.num_frames = frames.size();
    frame.frame0 = 0;
    frame.frame1 = 0;
    frame.frame2 = 0;
    frame.frame3 = 0;
    {
        uint64_t frame_tmp = 0;
        MBED_ASSERT(frames[0].size() <= sizeof(frame_tmp));
        memcpy(&frame_tmp, frames[0].data(), frames[0].size());
        frame.frame0 = frame_tmp;
    }
    if(frames.size() >= 1) { //NOLINT
        uint64_t frame_tmp = 0;
        MBED_ASSERT(frames[1].size() <= sizeof(frame_tmp));
        memcpy(&frame_tmp, frames[1].data(), frames[1].size());  
        frame.frame1 = frame_tmp;      
    }  //NOLINT
    if(frames.size() >= 2) { 
        uint64_t frame_tmp = 0;
        MBED_ASSERT(frames[1].size() <= sizeof(frame_tmp));
        memcpy(&frame_tmp, frames[1].data(), frames[1].size());  
        frame.frame1 = frame_tmp;      
    }
    if(frames.size() >= 3) {
        uint64_t frame_tmp = 0;
        MBED_ASSERT(frames[2].size() <= sizeof(frame_tmp));
        memcpy(&frame_tmp, frames[2].data(), frames[2].size());  
        frame.frame2 = frame_tmp;          
    }
    vector<uint8_t> pld(sizeof(T), 0);
    memcpy(pld.data(), (uint8_t *)(&frame), sizeof(T)); //NOLINT
    return pld;
}


template<class T> 
auto getFrames(vector<uint8_t> pld, const int bitrate) -> vector<vector<uint8_t>> {
    constexpr int BITS_IN_BYTE = 8;
    T frame{};
    int bytes_per_frame = static_cast<int>(ceilf((static_cast<float>(bitrate)/
            static_cast<float>(FRAME_RATE_HZ)/static_cast<float>(BITS_IN_BYTE))));
    vector<vector<uint8_t>> frames;
    // Copy in the initial values
    MBED_ASSERT(pld.size() == sizeof(T));
    memcpy(&frame, pld.data(), pld.size());
    MBED_ASSERT((frame.num_frames > 0) & (frame.num_frames <= 4));
    {
        uint64_t frame_tmp = frame.frame0;
        frames[0] = vector<uint8_t>(bytes_per_frame, 0);
        memcpy(frames[0].data(), &frame_tmp, bytes_per_frame);
    }
    if(frame.num_frames >= 1) { //NOLINT
        uint64_t frame_tmp = frame.frame1;
        frames[1] = vector<uint8_t>(bytes_per_frame, 1);
        memcpy(frames[1].data(), &frame_tmp, bytes_per_frame);   
    }  //NOLINT
    if(frame.num_frames >= 2) { 
        uint64_t frame_tmp = frame.frame2;
        frames[2] = vector<uint8_t>(bytes_per_frame, 0);
        memcpy(frames[2].data(), &frame_tmp, bytes_per_frame);   
    }
    if(frame.num_frames >= 3) {
        uint64_t frame_tmp = frame.frame2;
        frames[2] = vector<uint8_t>(bytes_per_frame, 0);
        memcpy(frames[2].data(), &frame_tmp, bytes_per_frame);     
    }
    return frames;
}


// Class that handles receiving voice frames and turning them
//  into the appropriate-sized data frames for transmit/receive.
class VoiceMsgProcessor {
private:
    vector<vector<uint8_t>> frames;

public:
    static auto valid_bitrate(const int bitrate) -> bool {
        return find(bitrates.begin(), bitrates.end(), bitrate) != bitrates.end();
    }

    static auto bits_per_frame(const int bitrate) -> int {
        MBED_ASSERT(valid_bitrate(bitrate));
        return bitrate/FRAME_RATE_HZ;
    }

    auto addFrame(const vector<uint8_t>& frame) -> bool;

    void clearFrames() {
        frames.clear();
    }

    auto getDataPayload() -> vector<uint8_t>;

    static auto getVoiceFrames(const vector<uint8_t>& pld) -> vector<vector<uint8_t>>;
};


#endif /* VOICE_MSG_HPP */