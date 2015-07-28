/* -*- c++ -*- */
/* 
 * Copyright 2015 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_IEEE802_11_CORR_SPREAMBLE_IMPL_H
#define INCLUDED_IEEE802_11_CORR_SPREAMBLE_IMPL_H

#include <ieee802-11/corr_spreamble.h>
#include <utils.h>
#include <gnuradio/filter/fft_filter.h>

//using namespace gr::ieee802_11;
//using namespace std;


namespace gr {
  namespace ieee802_11 {

    class corr_spreamble_impl : public corr_spreamble
    {
     private:
      pmt::pmt_t d_src_id;
      std::vector<gr_complex> d_symbols;
      float d_sps;
      float d_thresh, d_stashed_threshold;
      gr::filter::kernel::fft_filter_ccc *d_filter;

      gr_complex *d_corr;
      float *d_corr_mag;

      void _set_threshold(float threshold);


     public:
      corr_spreamble_impl(float threshold=0.9);
      ~corr_spreamble_impl();
      float threshold() const;
      void set_threshold(float threshold);

      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace ieee802-11
} // namespace gr

#endif /* INCLUDED_IEEE802_11_CORR_SPREAMBLE_IMPL_H */

