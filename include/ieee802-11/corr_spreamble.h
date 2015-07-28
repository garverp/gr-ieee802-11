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


#ifndef INCLUDED_IEEE802_11_CORR_SPREAMBLE_H
#define INCLUDED_IEEE802_11_CORR_SPREAMBLE_H

#include <ieee802-11/api.h>
#include <gnuradio/sync_block.h>

//using namespace gr::ieee802_11;
//using namespace std;
namespace gr{
   namespace ieee802_11{
    /*!
     * \brief <+description of block+>
     * \ingroup ieee802-11
     *
     */
    class IEEE802_11_API corr_spreamble : virtual public sync_block
    {
     public:
      typedef boost::shared_ptr<corr_spreamble> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of ieee802-11::corr_spreamble.
       *
       * To avoid accidental use of raw pointers, ieee802-11::corr_spreamble's
       * constructor is in a private implementation
       * class. ieee802-11::corr_spreamble::make is the public interface for
       * creating new instances.
       */
      static sptr make(float threshold=0.9);
      virtual float threshold() const = 0;
      virtual void set_threshold(float threshold) = 0;
    };

   }
}
#endif /* INCLUDED_IEEE802_11_CORR_SPREAMBLE_H */

