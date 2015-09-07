// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// Modified by Juan A. Corrales (ISIR, UPMC) in order to include coupling

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include <kdl_coupling/chain_coupling.hpp>
#include <ros/ros.h>

namespace KDL
{
  Chain_coupling::Chain_coupling() :
          Chain(),
          nrOfIndJoints(0),
          updateFunction(NULL),
          cm()
  {
  }

  Chain_coupling::Chain_coupling(const Chain_coupling &in) :
          Chain((const Chain &) in),
          nrOfIndJoints(0),
          updateFunction(NULL),
          cm()
  {
    this->setUpdateCouplingFunction(in.getUpdateCouplingFunction());
  }

  Chain_coupling::Chain_coupling(const Chain &in) :
          Chain(in),
          nrOfIndJoints(in.getNrOfJoints()),
          updateFunction(NULL),
          cm(Eigen::MatrixXd::Identity(in.getNrOfJoints(), in.getNrOfJoints()))
  {
  }

  Chain_coupling &Chain_coupling::operator=(const Chain_coupling &in)
  {
    // Copy KDL Chains
    Chain &c1 = (Chain &) (*this);
    const Chain &c2 = (const Chain &) (in);
    c1 = c2;
    // Copy Coupling Components (coupling matrix and coupling function)
    this->setUpdateCouplingFunction(in.getUpdateCouplingFunction());
    return *this;
  }

  Chain_coupling &Chain_coupling::operator=(const Chain &in)
  {
    // Copy KDL Chains
    Chain &c1 = (Chain &) (*this);
    c1 = in;
    // The number of independent joints is the same as the total number of joints
    this->nrOfIndJoints = in.getNrOfJoints();
    this->cm = Eigen::MatrixXd::Identity(in.getNrOfJoints(), in.getNrOfJoints());
    return *this;
  }

  bool Chain_coupling::isCoupled() const
  {
    return (nrOfIndJoints != this->getNrOfJoints());
  }

  bool Chain_coupling::setUpdateCouplingFunction(updateFuncPtr updateFunc)
  {
    if (updateFunc == NULL)
    {
      this->updateFunction = NULL;
      return 0;
    }
    // Verify that the update function generates a coupling matrix with correct size
    JntArray jnt(this->getNrOfJoints());
    KDL::SetToZero(jnt);
    Eigen::MatrixXd cm_init = updateFunc(jnt);
    if (cm_init.rows() != this->getNrOfJoints())
    {
      ROS_ERROR("Number of rows of coupling matrix has to match total number of joints");
      return -1;
    }
    // Initialize number of independent joints
    nrOfIndJoints = cm_init.cols();
    // Initialize update function
    this->updateFunction = updateFunc;
    // Initialize coupling matrix
    this->cm = cm_init;
    return 0;
  }

  updateFuncPtr Chain_coupling::getUpdateCouplingFunction() const
  {
    return this->updateFunction;
  }

  void Chain_coupling::updateCoupling(const JntArray &q)
  {
    if (this->updateFunction != NULL)
    {
      this->cm = this->updateFunction(q);
    }
  }

  Chain_coupling::~Chain_coupling()
  {
  }
}  // namespace KDL

