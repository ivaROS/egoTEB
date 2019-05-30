/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Christoph Rösmann, Franz Albers
 *********************************************************************/

#ifndef GAP_H_SIGNATURE_H_
#define GAP_H_SIGNATURE_H_

#include <teb_local_planner/equivalence_relations.h>
#include <teb_local_planner/misc.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/timed_elastic_band.h>

#include <ros/ros.h>
#include <math.h>
#include <algorithm>
#include <functional>
#include <vector>
#include <iterator>


namespace teb_local_planner
{

/**
 * @brief The H-signature defines an equivalence relation based on homology in terms of complex calculus.
 *
 * The H-Signature depends on the obstacle configuration and can be utilized
 * to check whether two trajectores belong to the same homology class.
 * Refer to: \n
 * 	- S. Bhattacharya et al.: Search-based Path Planning with Homotopy Class Constraints, AAAI, 2010
 */
class GapHSignature : public EquivalenceClass
{

public:

    /**
    * @brief Constructor accepting a TebConfig
    * @param cfg TebConfig storing some user configuration options
    */
    GapHSignature(const TebConfig& cfg) : cfg_(&cfg) {}


   /**
    * @brief Calculate the H-Signature of a path
    *
    * The implemented function accepts generic path descriptions that are restricted to the following structure: \n
    * The path is composed of points T and is represented by a std::vector< T > or similar type (std::list, std::deque, ...). \n
    * Provide a unary function with the following signature <c> std::complex< long double > (const T& point_type) </c>
    * that returns a complex value for the position (Re(*)=x, Im(*)=y).
     *
    * T could also be a pointer type, if the passed function also accepts a const T* point_Type.
    *
    * @param path_start Iterator to the first element in the path
    * @param path_end Iterator to the last element in the path
    * @param obstacles obstacle container
    * @param fun_cplx_point function accepting the dereference iterator type and that returns the position as complex number.
    * @tparam BidirIter Bidirectional iterator type
    * @tparam Fun function of the form std::complex< long double > (const T& point_type)
    */
    template<typename BidirIter, typename Fun>
    void calculateHSignature(BidirIter path_start, BidirIter path_end, Fun fun_cplx_point, const std::vector<egocircle_utils::gap_finding::Gap>& gaps)
    {
        // Do nothing
    }


   /**
    * @brief Check if two candidate classes are equivalent
    * @param other The other equivalence class to test with
    */
    virtual bool isEqual(const EquivalenceClass& other) const
    {
        return false;
    }

   /**
    * @brief Check if the equivalence value is detected correctly
    * @return Returns false, if the equivalence class detection failed, e.g. if nan- or inf values occur.
    */
    virtual bool isValid() const
    {
        true;
    }

    /**
     * @brief Check if the trajectory is non-looping around an obstacle.
     * @return Returns always true, as this cannot be detected by this kind of H-Signature.
     */
    virtual bool isReasonable() const
    {
      return true;
    }


private:

    const TebConfig* cfg_;
};


} // namespace teb_local_planner


#endif /* GAP_H_SIGNATURE_H_ */
