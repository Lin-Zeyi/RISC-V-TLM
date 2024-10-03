/*!
 \file Timer.cpp
 \brief Basic TLM-2 Timer module
 \author Màrius Montón
 \date January 2019
 */
// SPDX-License-Identifier: GPL-3.0-or-later

#include "Timer.h"
#include <cstdint>

namespace riscv_tlm::peripherals {

    SC_HAS_PROCESS(Timer);

    Timer::Timer(sc_core::sc_module_name const &name) :
            sc_module(name), socket("timer_socket"), m_mtime(0), m_mtimecmp(0) {

        socket.register_b_transport(this, &Timer::b_transport);

        SC_THREAD(run);
    }

    [[noreturn]] void Timer::run() {

        auto *irq_trans = new tlm::tlm_generic_payload;
        sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
        std::uint32_t cause = 1 << 31 | 0x07;     // Machine timer interrupt
        irq_trans->set_command(tlm::TLM_WRITE_COMMAND);
        irq_trans->set_data_ptr(reinterpret_cast<unsigned char *>(&cause));
        irq_trans->set_data_length(4);
        irq_trans->set_streaming_width(4);
        irq_trans->set_byte_enable_ptr(nullptr);
        irq_trans->set_dmi_allowed(false);
        irq_trans->set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
        irq_trans->set_address(0);

        // 每次进入循环后，如果事件没有触发，就会一直wait。如果事件触发了，就会调用b_transport方法发送中断，然后重新进入循环，wait下次触发。
        // 应该是只有循环体内部的部分会多次执行，外部的只会执行一次。
        while (true) {
            wait(timer_event);  // 如果没有用event，那么会在每个时间步触发。（可能是时钟周期，或者是指定仿真时间之类。）
            // 这个irq_line就是一个initial socket，向cpu的target socket发送trans。
            irq_line->b_transport(*irq_trans, delay);
        }
    }

    void Timer::b_transport(tlm::tlm_generic_payload &trans,
                            sc_core::sc_time &delay) {

        // 收取trans发来的配置信息
        tlm::tlm_command cmd = trans.get_command();
        sc_dt::uint64 addr = trans.get_address();
        unsigned char *ptr = trans.get_data_ptr();
        unsigned int len = trans.get_data_length();
        delay = sc_core::SC_ZERO_TIME;

        std::uint32_t aux_value = 0;

        // 如果是写命令，抽取出有效信息，写到类的私有变量里（有点类似写入寄存器）
        // 这块具体可以看通用净核对象的属性那一节。
        if (cmd == tlm::TLM_WRITE_COMMAND) {
            memcpy(&aux_value, ptr, len);
            switch (addr) {
                case TIMER_MEMORY_ADDRESS_LO:
                    m_mtime.range(31, 0) = aux_value;
                    break;
                case TIMER_MEMORY_ADDRESS_HI:
                    m_mtime.range(63, 32) = aux_value;
                    break;
                case TIMERCMP_MEMORY_ADDRESS_LO:
                    m_mtimecmp.range(31, 0) = aux_value;
                    break;
                case TIMERCMP_MEMORY_ADDRESS_HI:
                    m_mtimecmp.range(63, 32) = aux_value;

                    std::uint64_t notify_time;
                    // notify needs relative time, mtimecmp works in absolute time
                    notify_time = m_mtimecmp - m_mtime;

                    //timer_event.notify(sc_core::sc_time(notify_time, sc_core::SC_NS));
                    timer_event.notify(sc_core::sc_time::from_value(notify_time));
                    break;
                default:
                    trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
                    return;
            }
        } else { // TLM_READ_COMMAND
            switch (addr) {
                case TIMER_MEMORY_ADDRESS_LO:
                    m_mtime = sc_core::sc_time_stamp().value();
                    aux_value = m_mtime.range(31, 0);
                    break;
                case TIMER_MEMORY_ADDRESS_HI:
                    aux_value = m_mtime.range(63, 32);
                    break;
                case TIMERCMP_MEMORY_ADDRESS_LO:
                    aux_value = m_mtimecmp.range(31, 0);
                    break;
                case TIMERCMP_MEMORY_ADDRESS_HI:
                    aux_value = m_mtimecmp.range(63, 32);
                    break;
                default:
                    trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
                    return;
            }
            memcpy(ptr, &aux_value, len);
        }

        trans.set_response_status(tlm::TLM_OK_RESPONSE);
    }
}