/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>

#include "object_manipulation_msgs/GraspResult.h"
#include "object_manipulation_msgs/PlaceLocationResult.h"

namespace object_manipulation_msgs {

inline std::string getGraspResultInfo(GraspResult result)
{
  switch (result.result_code)
  {
  case GraspResult::SUCCESS: return ("grasp success");
  case GraspResult::GRASP_OUT_OF_REACH: return ("grasp out of reach");
  case GraspResult::GRASP_IN_COLLISION: return ("grasp in collision");
  case GraspResult::GRASP_UNFEASIBLE: return ("grasp unfeasible");
  case GraspResult::PREGRASP_OUT_OF_REACH: return ("pregrasp out of reach");
  case GraspResult::PREGRASP_IN_COLLISION: return ("pregrasp in collision");
  case GraspResult::PREGRASP_UNFEASIBLE: return ("pregrasp unfeasible");
  case GraspResult::LIFT_OUT_OF_REACH: return ("lift position out of reach");
  case GraspResult::LIFT_IN_COLLISION: return ("lift position in collision");
  case GraspResult::LIFT_UNFEASIBLE: return ("lift position unfeasible");
  case GraspResult::MOVE_ARM_FAILED: return ("arm movement failed");
  case GraspResult::GRASP_FAILED: return ("grasp failed");
  case GraspResult::LIFT_FAILED: return ("lift failed");
  case GraspResult::RETREAT_FAILED: return ("retreat failed");
  default: return("unknown result code returned");
  }
}

inline std::string getPlaceLocationResultInfo(PlaceLocationResult result)
{
  switch (result.result_code)
  {
  case PlaceLocationResult::SUCCESS: return ("place success");
  case PlaceLocationResult::PLACE_OUT_OF_REACH: return ("place location out of reach");
  case PlaceLocationResult::PLACE_IN_COLLISION: return ("place location in collision");
  case PlaceLocationResult::PLACE_UNFEASIBLE: return ("place in location unfeasible");
  case PlaceLocationResult::PREPLACE_OUT_OF_REACH: return ("preplace location out of reach");
  case PlaceLocationResult::PREPLACE_IN_COLLISION: return ("preplace location in collision");
  case PlaceLocationResult::PREPLACE_UNFEASIBLE: return ("preplace location unfeasible");
  case PlaceLocationResult::RETREAT_OUT_OF_REACH: return ("retreat location out of reach");
  case PlaceLocationResult::RETREAT_IN_COLLISION: return ("retreat location in collision");
  case PlaceLocationResult::RETREAT_UNFEASIBLE: return ("retreat location unfeasible");
  case PlaceLocationResult::MOVE_ARM_FAILED: return ("arm movement failed");
  case PlaceLocationResult::PLACE_FAILED: return ("place failed");
  case PlaceLocationResult::RETREAT_FAILED: return ("retreat failed");
  default: return("unknown result code returned");
  }
}

}
