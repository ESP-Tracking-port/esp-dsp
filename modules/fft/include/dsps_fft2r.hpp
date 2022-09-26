// Copyright 2018-2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _dsps_fft2r_HPP_
#define _dsps_fft2r_HPP_

#include "dsp_err.h"
#include "sdkconfig.h"
#include "dsps_fft_tables.h"
#include "dsps_fft2r_platform.h"
#include "dsps_fft2r.h"
#include "dsp_common.h"
#include "dsp_types.h"
#include <math.h>
#include "esp_attr.h"

constexpr int add_rount_mult = 0x7fff;
constexpr int mult_shift_const = 0x7fff; // Used to shift data << 15

static inline int16_t xtfixed_bf_1(int16_t a0, int16_t a1, int16_t a2, int16_t a3, int16_t a4, int result_shift)
{
	int result = a0*mult_shift_const;
	result -= (int32_t)a1*(int32_t)a2 + (int32_t)a3*(int32_t)a4;
	result += add_rount_mult;
	result = result >> result_shift;
	return (int16_t)result;
}

static inline int16_t xtfixed_bf_2(int16_t a0, int16_t a1, int16_t a2, int16_t a3, int16_t a4, int result_shift)
{
	int result = a0 *mult_shift_const;
	result -= ((int32_t)a1*(int32_t)a2 - (int32_t)a3*(int32_t)a4);
	result += add_rount_mult;
	result = result >> result_shift;
	return (int16_t)result;
}

static inline int16_t xtfixed_bf_3(int16_t a0, int16_t a1, int16_t a2, int16_t a3, int16_t a4, int result_shift)
{
	int result = a0 *mult_shift_const;
	result += (int32_t)a1*(int32_t)a2 + (int32_t)a3*(int32_t)a4;
	result += add_rount_mult;
	result = result >> result_shift;
	return (int16_t)result;
}

static inline int16_t xtfixed_bf_4(int16_t a0, int16_t a1, int16_t a2, int16_t a3, int16_t a4, int result_shift)
{
	int result = a0 *mult_shift_const;
	result += (int32_t)a1*(int32_t)a2 - (int32_t)a3*(int32_t)a4;
	result += add_rount_mult;
	result = result >> result_shift;
	return (int16_t)result;
}

/// \tparam T An entity mocking a pointer type through implementing operator[].
/// \arg in_data Input buffer storing signal in ReImReIm layout
///
template <class T>
esp_err_t dsps_fft2r_sc16_ansi_step(T in_data, int N, int16_t* sc_table)
{
    if (!dsp_is_power_of_two(N)) {
        return ESP_ERR_DSP_INVALID_LENGTH;
    }
    if (!dsps_fft2r_sc16_initialized) {
        return ESP_ERR_DSP_UNINITIALIZED;
    }

    esp_err_t result = ESP_OK;

    uint32_t *w = (uint32_t*)sc_table;

	int ie, ia, m;
	sc16_t cs;// c - re, s - im
	sc16_t m_data;
	sc16_t a_data;

	ie = 1;
	for (int N2 = N / 2; N2 > 0; N2 >>= 1) {
		ia = 0;
		for (int j = 0; j < ie; j++) {
			cs.data = w[j];
			//c = w[2 * j];
			//s = w[2 * j + 1];
			for (int i = 0; i < N2; i++) {
				m = ia + N2;
				m_data.data = in_data[m];
				a_data.data = in_data[ia];
				//data[2 * m] = data[2 * ia] - re_temp;
				//data[2 * m + 1] = data[2 * ia + 1] - im_temp;
				sc16_t m1;
				m1.re = xtfixed_bf_1(a_data.re, cs.re, m_data.re, cs.im, m_data.im, 16);//(a_data.re - temp.re + shift_const) >> 1;
				m1.im = xtfixed_bf_2(a_data.im, cs.re, m_data.im, cs.im, m_data.re, 16);//(a_data.im - temp.im + shift_const) >> 1;
				in_data[m] = m1.data;

				//data[2 * ia] = data[2 * ia] + re_temp;
				//data[2 * ia + 1] = data[2 * ia + 1] + im_temp;
				sc16_t m2;
				m2.re = xtfixed_bf_3(a_data.re, cs.re, m_data.re, cs.im, m_data.im, 16);//(a_data.re + temp.re + shift_const) >> 1;
				m2.im = xtfixed_bf_4(a_data.im, cs.re, m_data.im, cs.im, m_data.re, 16);//(a_data.im + temp.im + shift_const)>>1;
				in_data[ia] = m2.data;
				ia++;
			}
			ia += N2;
		}
		ie <<= 1;
	}
	return result;
}

template <class T>
esp_err_t dsps_fft2r_fc32_ansi_step(T *data, int N, float *w)
{
    if (!dsp_is_power_of_two(N)) {
        return ESP_ERR_DSP_INVALID_LENGTH;
    }
    if (!dsps_fft2r_initialized) {
        return ESP_ERR_DSP_UNINITIALIZED;
    }

    esp_err_t result = ESP_OK;

    int ie, ia, m;
    float re_temp, im_temp;
    float c, s;
    ie = 1;
    for (int N2 = N / 2; N2 > 0; N2 >>= 1) {
        ia = 0;
        for (int j = 0; j < ie; j++) {
            c = w[2 * j];
            s = w[2 * j + 1];
            for (int i = 0; i < N2; i++) {
                m = ia + N2;
                re_temp = c * data[2 * m] + s * data[2 * m + 1];
                im_temp = c * data[2 * m + 1] - s * data[2 * m];
                data[2 * m] = data[2 * ia] - re_temp;
                data[2 * m + 1] = data[2 * ia + 1] - im_temp;
                data[2 * ia] = data[2 * ia] + re_temp;
                data[2 * ia + 1] = data[2 * ia + 1] + im_temp;
                ia++;
            }
            ia += N2;
        }
        ie <<= 1;
    }
    return result;
}

#endif // _dsps_fft2r_HPP_
