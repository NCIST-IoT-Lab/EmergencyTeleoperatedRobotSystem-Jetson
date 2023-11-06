//
// Create by HoChihchou on 2023/5/6
//

#ifndef _SOUND_SOURCE_LOCALIZATION_CASWEBSOCKET_H_
#define _SOUND_SOURCE_LOCALIZATION_CASWEBSOCKET_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Utility.h>
#include <alsa/asoundlib.h>
#include <iostream>

using namespace std;
using namespace etrs::utility;

namespace etrs::ssl {
    class SoundSourceDetector {
    private:
        unsigned int sample_rate; // 采样率
        int samples;              // 每个周期的采样数
        int channels;             // 声道数
        snd_pcm_format_t format;  // 数据格式为float类型
        unsigned long period_size;
        int fmt_size;
        int buffer_size;
        string microphone_name;
        snd_pcm_t *pcm_handle;
        snd_pcm_hw_params_t *hw_params;
        snd_pcm_sw_params_t *sw_params;

    public:
        explicit SoundSourceDetector(unsigned int smaple_rate, const int samples, const int channels,
                                     string microphone_name);
        ~SoundSourceDetector();
        // 开始检测†
        bool start();
        // 停止检测
        void stop();
        // 获取一次声源位置
        Eigen::Vector3f locate();
    };
} // namespace etrs::ssl

#endif // _SOUND_SOURCE_LOCALIZATION_CASWEBSOCKET_H_
