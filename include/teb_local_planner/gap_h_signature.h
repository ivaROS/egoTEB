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
#include <teb_local_planner/egocircle_interface.h>
#include <teb_local_planner/gap_intersection.h>

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

    
    
    
//     {
//       using ego_circle::PolarPoint;
//       using ego_circle::EgoCircularPoint;
//       
//       EgoCircularPoint p1(pose1->pose().position().x(), pose1->pose().position().y());
//       egocircle->toLocal(p1);
//       PolarPoint pp1(p1); 
//       
//       EgoCircularPoint p2(pose2->pose().position().x(), pose2->pose().position().y());
//       egocircle->toLocal(p2);
//       PolarPoint pp2(p2);
//       
//       ROS_DEBUG_STREAM("gap_signature", "Gap: " "P1(" << p1.x << "," << p1.y << "): [" << pp1.r << "m @" << pp1.theta << "] P2(" << p2.x << "," << p2.y << "): [" << pp2.r << "m @" << pp2.theta << "]");
//       
//       double dist = -1;
//       if(pp1.theta >= gap.start.theta && pp1.theta <= gap.end.theta && pp2.theta >= gap.start.theta && pp2.theta <= gap.end.theta)
//       {
//         double r1e = (pp1.theta - gap.start.theta)/(gap.end.theta - gap.start.theta)*(gap.end.r - gap.start.r)+gap.start.r;
//         double r2e = (pp2.theta - gap.start.theta)/(gap.end.theta - gap.start.theta)*(gap.end.r - gap.start.r)+gap.start.r;
//         
//         double avgr = (r1e + r2e)/2;
//         
//         if(pp2.r >= avgr && pp1.r <= avgr)
//         {
//           dist = (pp2.r - avgr) + (avgr - pp1.r);
//         }
//         
//       }
//       return dist;
//       
//     }

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
    template<typename BidirIter>
    void calculateHSignature(BidirIter path_start, BidirIter path_end, const EgoCircleInterface* egocircle)
    {

      std::advance(path_end, -1); // reduce path_end by 1 (since we check line segments between those path points
      
      
      const auto& gaps = egocircle->getGlobalGaps();
      const int num_gaps = gaps.size();
      
      // iterate path
      while(path_start != path_end)
      {
        auto v1 = *path_start;
        auto v2 = *boost::next(path_start);
        
        for (std::size_t l=0; l<num_gaps; ++l) // iterate all obstacles
        {
          auto gap = gaps[l];
          Eigen::Vector2d p;
          bool it = intersects(gap, v1, v2, &p);
          if(it)
          {
            gap_number_ = l;
            ROS_INFO_STREAM("Trajectory passes through gap #" << l << " at [" << p.x() << "," << p.y() << "]");
            return;
          }
        }
        ++path_start;
      }
      gap_number_ = -1;
      ROS_INFO_STREAM("Trajectory does not appear to pass through any gap, so giving it index #" << gap_number_);
    }


    

   /**
    * @brief Check if two candidate classes are equivalent
    * @param other The other equivalence class to test with
    */
    virtual bool isEqual(const EquivalenceClass& other) const
    {
      const GapHSignature* hother = dynamic_cast<const GapHSignature*>(&other);
      if (hother)
      {
        return hother->gap_number_ == gap_number_;
      }
      else
        ROS_ERROR("Cannot compare GapHSignature equivalence classes with types other than GapHSignature.");
      
        //return false;
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

    int getGap() const
    {
      return gap_number_;
    }

private:
    int gap_number_;
    const TebConfig* cfg_;
};

} // namespace teb_local_planner


#endif /* GAP_H_SIGNATURE_H_ */
