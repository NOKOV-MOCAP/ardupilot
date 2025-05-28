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
 * AP_C610CAN.cpp
 *
 *      Author: Francisco Ferreira and Tom Pittenger
 */

#include "AP_C610CAN.h"

#if AP_C610CAN_ENABLED
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>    // for MIN,MAX

extern const AP_HAL::HAL& hal;

#define AP_C610CAN_DEBUG 0
int16_t pwm_to_current(uint16_t pwm);
// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_C610CAN::var_info[] = {

    // @Param: NPOLE
    // @DisplayName: Number of motor poles
    // @Description: Sets the number of motor poles to calculate the correct RPM value
    AP_GROUPINFO("NPOLE", 1, AP_C610CAN, _num_poles, DEFAULT_NUM_POLES),

    AP_GROUPEND
};

AP_C610CAN::AP_C610CAN()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_C610CAN must be singleton");
    }
#endif
    _singleton = this;
}

void AP_C610CAN::init()
{
    if (_driver != nullptr) {
        // only allow one instance
        return;
    }

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::C610CAN) {
            _driver = NEW_NOTHROW AP_C610CAN_Driver();
            return;
        }
    }
}

void AP_C610CAN::update()
{
    if (_driver == nullptr) {
        return;
    }
    _driver->update((uint8_t)_num_poles.get());
}

AP_C610CAN_Driver::AP_C610CAN_Driver() : CANSensor("C610CAN")
{
    register_driver(AP_CAN::Protocol::C610CAN);

    // start thread for receiving and sending CAN frames. Tests show we use about 640 bytes of stack
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_C610CAN_Driver::loop, void), "c610can", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0);
}

void AP_C610CAN_Driver::handle_frame(AP_HAL::CANFrame &frame)
{
    // 只处理标准数据帧
    if (frame.isExtended()) {
        return;
    }

    // 检查是否是已知电调反馈ID
    bool valid_id = false;
    uint8_t esc_idx = 0;
    for (; esc_idx < 4; esc_idx++) {
        if (frame.id == esc_id_map[esc_idx]) {
            valid_id = true;
            break;
        }
    }
    
    if (!valid_id || frame.dlc != 8) {
        return;
    }

    // 解析反馈数据 (小端格式)
     __attribute__((unused)) const uint16_t angle = (frame.data[1] << 8) | frame.data[0]; 
    const int16_t rpm = frame.data[2] << 8 | frame.data[3];
    const int16_t current = frame.data[4] << 8 | frame.data[5];
    const int16_t temp = frame.data[6] << 8 | frame.data[7];

    // 更新电调状态
    update_rpm(esc_idx, abs(rpm));

    // 转换温度单位(假设原始单位为°C)
    const int16_t temp_cdeg = temp * 100;
    
    // 转换电流单位(假设原始单位为0.01A)
    const float current_amps = current * 0.01f;

    const TelemetryData t {
        .temperature_cdeg = temp_cdeg,
        .voltage = 0, // C610协议未提供电压
        .current = current_amps
    };
    
    update_telem_data(esc_idx, t,
        AP_ESC_Telem_Backend::TelemetryType::CURRENT |
        AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);

    // 标记电调为已检测到
    if (!(_init.detected_bitmask & (1 << esc_idx))) {
        _init.detected_bitmask |= (1 << esc_idx);
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "C610CAN: Found ESC ID 0x%03X at index %d", frame.id, esc_idx);
    }
}

void AP_C610CAN_Driver::update(const uint8_t num_poles)
{
    if (_init.detected_bitmask == 0) {
        // nothing to do...
        return;
    }
    
    WITH_SEMAPHORE(_output.sem);
    for (uint8_t i = 0; i < ARRAY_SIZE(_output.pwm); i++) {
        if ((_init.detected_bitmask & (1UL<<i)) == 0 || SRV_Channels::channel_function(i) <= SRV_Channel::Function::k_none) {
            _output.pwm[i] = 0;
            continue;
        }

        const SRV_Channel *c = SRV_Channels::srv_channel(i);
        if (c == nullptr) {
            _output.pwm[i] = 0;
            continue;
        }
        _output.pwm[i] = c->get_output_pwm();
    }

    _output.is_new = true;

#if AP_C610CAN_USE_EVENTS
    if (_output.thread_ctx != nullptr) {
        // trigger the thread to wake up immediately
        chEvtSignal(_output.thread_ctx, 1);
    }
#endif
}


void AP_C610CAN_Driver::loop()
{
    uint16_t pwm[ARRAY_SIZE(_output.pwm)] {};
#if AP_C610CAN_USE_EVENTS
    _output.thread_ctx = chThdGetSelfX();
#endif
    while (true) {

#if AP_C610CAN_USE_EVENTS
        // sleep until we get new data, but also wake up at 1KHz to send the old data again
        chEvtWaitAnyTimeout(ALL_EVENTS, chTimeUS2I(2500));
 #else
        hal.scheduler->delay_microseconds(2500); // 1KHz
#endif
        uint32_t now = AP_HAL::millis();
        
        // 1. 更新PWM值
        {
            WITH_SEMAPHORE(_output.sem);
            if (_output.is_new) {
                _output.last_new_ms = now;
                _output.is_new = false;
                memcpy(&pwm, &_output.pwm, sizeof(pwm));
            }
            
            // 超时清零
            if (_output.last_new_ms && now - _output.last_new_ms > 1000) {
                memset(&pwm, 0, sizeof(pwm));
                _output.last_new_ms = 0;
            }
        }


        // 2. 以1kHz频率发送控制命令
        send_control_command(pwm,1000);
        
        // 3. 维持1kHz循环频率
        // hal.scheduler->delay_microseconds(1000);
    }
}

int16_t pwm_to_current(uint16_t pwm) {
    
    // 线性映射公式：(PWM - 1500) * 40
    // uint16_t incr_pwm = 50;
    // if(pwm > 1450 && pwm <1500){
    //     pwm -= incr_pwm;
    // }
    // if(pwm < 1550 && pwm >1500){
    //     pwm += incr_pwm;
    // }
    // 确保PWM在有效范围内（安全保护）
    pwm = constrain_int16(pwm, 1000, 2000);
    return static_cast<int16_t>((pwm - 1500) * 6);
}

bool AP_C610CAN_Driver::send_control_command(uint16_t pwm[],uint32_t timeout_us)
{
    AP_HAL::CANFrame frame;
    frame.id = C610_CTRL_BASE_ID; // 使用0x200作为控制ID
    //frame.isExtended(false);
    frame.dlc = 8;

    // 将PWM值(1000~2000)转换为电流值(-10000~10000)
    int16_t currents[4];
    for (uint8_t i = 0; i < 4; i++) {
        if (_init.detected_bitmask & (1 << i)) {
            // gcs().send_text(MAV_SEVERITY_WARNING, "servo%d: %d",i+1,pwm[i]);
            currents[i] = constrain_int16(pwm_to_current(pwm[i]), -10000, 10000);
            if(i==1||i==3)
                currents[i] *= -1;
        } else {
            currents[i] = 0;
        }
        frame.data[i*2] = currents[i] >> 8;
        frame.data[i*2+1] = currents[i] & 0xFF;
    }  

    return write_frame(frame, timeout_us);
}

// singleton instance
AP_C610CAN *AP_C610CAN::_singleton;

namespace AP {
AP_C610CAN *c610can()
{
    return AP_C610CAN::get_singleton();
}
};

#endif // AP_C610CAN_ENABLED

