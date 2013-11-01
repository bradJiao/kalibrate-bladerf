/*
 * Copyright (c) 2010, Joshua Lackey
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     *  Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *
 *     *  Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <stdio.h>
#include <stdlib.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include <string.h>
#include <pthread.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <complex>

#include "usrp_source.h"

extern int g_verbosity;


#ifdef _WIN32
inline double round(double x) { return floor(x + 0.5); }
#endif
inline unsigned int min(unsigned int a, unsigned int b){return a>b?b:a; }

usrp_source::usrp_source(float sample_rate, long int fpga_master_clock_freq) {

	m_fpga_master_clock_freq = fpga_master_clock_freq;
	m_desired_sample_rate = sample_rate;
	m_sample_rate = 0.0;
	m_decimation = 0;
	m_cb = new circular_buffer(CB_LEN, sizeof(complex), 0);

	pthread_mutex_init(&m_u_mutex, 0);
}


usrp_source::usrp_source(unsigned int decimation, long int fpga_master_clock_freq) {

	m_fpga_master_clock_freq = fpga_master_clock_freq;
	m_sample_rate = 0.0;
	m_cb = new circular_buffer(CB_LEN, sizeof(complex), 0);

	pthread_mutex_init(&m_u_mutex, 0);

	m_decimation = decimation & ~1;
	if(m_decimation < 4)
		m_decimation = 4;
	if(m_decimation > 256)
		m_decimation = 256;
}


usrp_source::~usrp_source() {

	stop();
	delete m_cb;
	bladerf_close(dev);
	pthread_mutex_destroy(&m_u_mutex);
}


void usrp_source::stop() {

	pthread_mutex_lock(&m_u_mutex);
	pthread_mutex_unlock(&m_u_mutex);
}


void usrp_source::start() {

	pthread_mutex_lock(&m_u_mutex);
	pthread_mutex_unlock(&m_u_mutex);
}


void usrp_source::calculate_decimation() {

	float decimation_f;

//	decimation_f = (float)m_u_rx->fpga_master_clock_freq() / m_desired_sample_rate;
	m_decimation = (unsigned int)round(decimation_f) & ~1;

	if(m_decimation < 4)
		m_decimation = 4;
	if(m_decimation > 256)
		m_decimation = 256;
}


float usrp_source::sample_rate() {

	return m_sample_rate;

}


int usrp_source::tune(double freq) {

	int r = 0;

	pthread_mutex_lock(&m_u_mutex);
	if (freq != m_center_freq) {
		r = bladerf_set_frequency( dev, BLADERF_MODULE_RX, (uint32_t)freq );
		//r = rtlsdr_set_center_freq(dev, (uint32_t)freq);
//		fprintf(stderr, "Tuned to %i Hz.\n", (uint32_t)freq);

		if (r < 0)
			fprintf(stderr, "Tuning to %lu Hz failed!\n", (uint32_t)freq);
		else
			m_center_freq = freq;
	}

	pthread_mutex_unlock(&m_u_mutex);

	return 1; //(r < 0) ? 0 : 1;
}

int usrp_source::set_freq_correction(int ppm) {
	m_freq_corr = ppm;
	return 0;//rtlsdr_set_freq_correction(dev, ppm);
}

bool usrp_source::set_antenna(int antenna) {

	return 0;
}

bool usrp_source::set_gain(float gain) {
	return 0;
}


/*
 * open() should be called before multiple threads access usrp_source.
 */
int usrp_source::open(unsigned int subdev) {

	uint32_t samp_rate = 270833;

	m_sample_rate = 270833.002142;

	int status = 0;
	if(!dev) {
		status = bladerf_open(&dev, NULL);// open default device
		if (status) {
			fprintf(stderr, "Failed to open device : %s\n", bladerf_strerror(status));
			status = -1; 
		}   

		fprintf(stderr, "Using nuand LLC bladeRF \n" );

		char serial[BLADERF_SERIAL_LENGTH];
		if ( bladerf_get_serial( dev, serial ) == 0 ) 
			fprintf(stderr, " SN %s",serial );
		//std::cerr << " SN " << serial;

		struct bladerf_version ver;
		if ( bladerf_fw_version( dev, &ver ) == 0 ) 
			fprintf(stderr, " FW v%i.%i.%i\n",ver.major,ver.minor,ver.patch);
		//std::cerr << " FW v" << ver.major << "." << ver.minor << "." << ver.patch;


		if ( bladerf_is_fpga_configured( dev ) != 1 ) 
		{   
			fprintf(stderr, "The FPGA is not configured! \n");
			return -1; 
		}   


		bladerf_enable_module(dev,BLADERF_MODULE_RX, true);

		status = bladerf_set_sample_rate(dev,BLADERF_MODULE_RX, samp_rate, &m_sample_rate);

		fprintf(stderr, "Sample rate: %d\n", m_sample_rate);
	}




	/* Reset endpoint before we start reading from it (mandatory) */
	//r = rtlsdr_reset_buffer(dev);
	//if (r < 0)
		//fprintf(stderr, "WARNING: Failed to reset buffers.\n");

	//	r = rtlsdr_set_offset_tuning(dev, 1);
	//	if (r < 0)
	//		fprintf(stderr, "WARNING: Failed to enable offset tuning\n");

	return 0;
}

#define USB_PACKET_SIZE		(16 * 1024)
#define FLUSH_SIZE		512

unsigned short ubuf[USB_PACKET_SIZE*2];

int usrp_source::fill(unsigned int num_samples, unsigned int *overrun_i) {

	unsigned int i, j, space, overruns = 0;
	complex *c;
	int n_read;
	struct bladerf_metadata metadata;

	while((m_cb->data_available() < num_samples) && (m_cb->space_available() > 0)) {

		// read one usb packet from the usrp
		pthread_mutex_lock(&m_u_mutex);

		if (bladerf_rx(dev,BLADERF_FORMAT_SC16_Q12, ubuf, sizeof(ubuf)/2, &metadata) < 0) {
			pthread_mutex_unlock(&m_u_mutex);
			fprintf(stderr, "error: usrp_standard_rx::read\n");
			return -1;
		}

		pthread_mutex_unlock(&m_u_mutex);

		// write complex<short> input to complex<float> output
		c = (complex *)m_cb->poke(&space);

		// set space to number of complex items to copy
		space = min(sizeof(ubuf)/2, num_samples);

		// write data
		for(i = 0, j = 0; i < space; i += 1, j += 2)
			c[i] = complex((ubuf[j] - 2047) * 16, (ubuf[j + 1] - 2047) * 16);

		// update cb
		m_cb->wrote(i);
	}

	// if the cb is full, we left behind data from the usb packet
	if(m_cb->space_available() == 0) {
		fprintf(stderr, "warning: local overrun\n");
		overruns++;
	}

	return 0;
}


int usrp_source::read(complex *buf, unsigned int num_samples,
   unsigned int *samples_read) {

	unsigned int n;

	if(fill(num_samples, 0))
		return -1;

	n = m_cb->read(buf, num_samples);

	if(samples_read)
		*samples_read = n;

	return 0;
}


/*
 * Don't hold a lock on this and use the usrp at the same time.
 */
circular_buffer *usrp_source::get_buffer() {

	return m_cb;
}


int usrp_source::flush(unsigned int flush_count) {

	m_cb->flush();
	fill(flush_count * FLUSH_SIZE, 0);
	m_cb->flush();

	return 0;
}
