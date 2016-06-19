////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2011, Benjamin Cohen, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Benjamin Cohen
/// \author Andrew Dornbush

#ifndef sbpl_collision_robot_collision_model_h
#define sbpl_collision_robot_collision_model_h

// standard includes
#include <memory>
#include <string>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <Eigen/StdVector>

// project includes
#include <sbpl_collision_checking/collision_model_config.h>

namespace sbpl {
namespace collision {

typedef Eigen::aligned_allocator<Eigen::Affine3d> Affine3dAllocator;
typedef std::vector<Eigen::Affine3d, Affine3dAllocator> Affine3dVector;

/// \brief Collision Sphere Model Specification
struct CollisionSphereModel
{
    std::string name;
    Eigen::Vector3d center; ///< offset from link center
    double radius;
    int priority;
};

/// \brief Collision Spheres Model Specification
struct CollisionSpheresModel
{
    int link_index;
    std::vector<const CollisionSphereModel*> spheres;
};

/// \brief Collision Voxels Model Specification
struct CollisionVoxelsModel
{
    int link_index;
    double voxel_res;
    std::vector<Eigen::Vector3d> voxels; // in the link frame
};

/// \brief Sphere Collision State Specification
struct CollisionSphereState
{
    Eigen::Vector3d pos;
};

struct CollisionSpheresState
{
    std::vector<CollisionSphereState*> states;
};

/// \brief Voxel Collision State Specification
struct CollisionVoxelsState
{
    std::vector<Eigen::Vector3d> voxels; // in the model frame
};

/// \brief Collision Group Model Specification
struct CollisionGroupModel
{
    std::string name;
    std::vector<int> link_indices;
};

/// \brief Collision Group State
struct CollisionGroupState
{
    std::vector<int> sphere_indices; ///< sphere states inside the group
    std::vector<int> voxels_indices; ///< voxels states outside the group
};

class CollisionModelImpl;

/// \brief Represents the collision model of the robot used for planning.
class RobotCollisionModel
{
public:

    RobotCollisionModel();
    ~RobotCollisionModel();

    bool init(
        const std::string& urdf_string,
        const CollisionModelConfig& config);

    /// \name Robot Model - General Information
    ///@{
    auto   name() const -> const std::string&;
    auto   modelFrame() const -> const std::string&;
    ///@}

    /// \name Robot Model - Joint Information
    ///@{
    size_t jointVarCount() const;
    auto   jointVarNames() const -> const std::vector<std::string>&;

    bool   hasJointVar(const std::string& joint_name) const;
    int    jointVarIndex(const std::string& joint_name) const;
    auto   jointVarName(int jidx) const -> const std::string&;

    bool   jointVarIsContinuous(const std::string& joint_name) const;
    bool   jointVarHasPositionBounds(const std::string& joint_name) const;
    double jointVarMaxPosition(const std::string& joint_name) const;
    double jointVarMinPosition(const std::string& joint_name) const;

    bool   jointVarIsContinuous(int jidx) const;
    bool   jointVarHasPositionBounds(int jidx) const;
    double jointVarMinPosition(int jidx) const;
    double jointVarMaxPosition(int jidx) const;
    ///@}

    /// \name Robot Model - Link Information
    ///@{
    size_t linkCount() const;
    auto   linkNames() const -> const std::vector<std::string>&;

    bool  hasLink(const std::string& link_name) const;
    int   linkIndex(const std::string& link_name) const;
    auto  linkName(int lidx) const -> const std::string&;
    ///@}

    /// \name Collision Model - Collision Spheres Information
    ///@{

    /// \brief Return the number of sphere collision models
    size_t sphereModelCount() const;

    /// \brief Return the sphere collision model for a given sphere collision
    ///        state
    auto   sphereModel(int smidx) const -> const CollisionSphereModel&;

    ///@}

    /// \name Collision Model - Collision Spheres Model Information
    ///@{
    bool hasSpheresModel(const std::string& link_name) const;
    bool hasSpheresModel(int lidx) const;
    ///@}

    /// \name Collision Model - Collision Voxels Model Information
    ///@{

    bool hasVoxelsModel(const std::string& link_name) const;
    bool hasVoxelsModel(int lidx) const;

    /// \brief Return the number of voxel model collision states
    size_t voxelsModelCount() const;

    /// \brief Return the voxel collision model for a given voxel collision state
    auto   voxelsModel(int vmidx) const -> const CollisionVoxelsModel&;

    ///@}

    /// \name Collision Model - Group Information
    ///@{

    /// \brief Return the number of collision groups
    size_t groupCount() const;

    /// \brief Return the collision groups
    auto   groups() const -> const std::vector<CollisionGroupModel>&;

    /// \brief Return whether a collision group exists within the model
    bool   hasGroup(const std::string& group_name) const;

    /// \brief Return the index for a collision group
    int    groupIndex(const std::string& group_name) const;

    /// \brief Return the name of a collision group from its index
    auto   groupName(int gidx) const -> const std::string&;

    /// \brief Return the indices of the links belonging to a group
    auto   groupLinkIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto   groupLinkIndices(int gidx) const ->
            const std::vector<int>&;

    /// \brief Return the indices of the collision sphere states belonging to
    ///        this group
    auto   groupSphereStateIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto   groupSphereStateIndices(int gidx) const ->
            const std::vector<int>&;

    /// \brief Return the indices of the collision voxels states NOT belonging
    ///        to this group.
    auto  groupOutsideVoxelsStateIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto   groupOutsideVoxelsStateIndices(int gidx) const ->
            const std::vector<int>&;

    ///@}

    /// \name RobotState
    ///@{
    auto   worldToModelTransform() const -> const Eigen::Affine3d&;
    bool   setWorldToModelTransform(const Eigen::Affine3d& transform);

    auto   jointPositions() const -> const std::vector<double>&;
    auto   linkTransforms() const -> const Affine3dVector&;

    double jointPosition(const std::string& joint_name) const;
    double jointPosition(int jidx) const;

    bool   setJointPosition(const std::string& name, double position);
    bool   setJointPosition(int jidx, double position);

    auto   linkTransform(const std::string& link_name) const ->
            const Eigen::Affine3d&;
    auto   linkTransform(int lidx) const -> const Eigen::Affine3d&;

    bool   linkTransformDirty(const std::string& link_name) const;
    bool   linkTransformDirty(int lidx) const;

    bool   updateLinkTransforms();
    bool   updateLinkTransform(int lidx);
    bool   updateLinkTransform(const std::string& link_name);
    ///@}

    /// \name CollisionState
    ///@{
    auto voxelsState(int vsidx) const -> const CollisionVoxelsState&;
    bool voxelsStateDirty(int vsidx) const;
    bool updateVoxelsStates();
    bool updateVoxelsState(int vsidx);

    auto sphereState(int ssidx) const -> const CollisionSphereState&;
    bool sphereStateDirty(int ssidx) const;
    bool updateSpherePositions();
    bool updateSpherePosition(int ssidx);
    ///@}

private:

    std::unique_ptr<CollisionModelImpl> m_impl;
};

} // namespace collision
} // namespace sbpl

#endif
