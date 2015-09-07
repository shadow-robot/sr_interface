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

#include <kdl_coupling/chainjnttojacsolver_coupling.hpp>
namespace KDL
{
  ChainJntToJacSolver_coupling::ChainJntToJacSolver_coupling(const Chain_coupling& _chain):
    chain(_chain),jac_tmp(chain.getNrOfJoints())
    {
    }

    ChainJntToJacSolver_coupling::~ChainJntToJacSolver_coupling()
    {
    }

    int ChainJntToJacSolver_coupling::JntToJac(const JntArray& q_in,Jacobian& jac)
    {
        unsigned int jac_columns=chain.getNrOfIndJoints();

        if(q_in.rows()!=chain.getNrOfJoints()||jac_columns!=jac.columns())
            return -1;

        T_tmp = Frame::Identity();
        SetToZero(t_tmp);
        int j=0;
        int k=0;
        Frame total;
        for (unsigned int i=0;i<chain.getNrOfSegments();i++) {
           // Calculate new Frame_base_ee
            if(chain.getSegment(i).getJoint().getType()!=Joint::None){
            // pose of the new end-point expressed in the base
                total = T_tmp*chain.getSegment(i).pose(q_in(j));
               // changing base of new segment's twist to base frame
               // t_tmp = T_tmp.M*chain.getSegment(i).twist(1.0);
                t_tmp = T_tmp.M*chain.getSegment(i).twist(q_in(j),1.0);
            }else{
                total = T_tmp*chain.getSegment(i).pose(0.0);
            }

           // Changing Refpoint of all columns to new ee
            changeRefPoint(jac_tmp,total.p-T_tmp.p,jac_tmp);

           // Only increase jointnr if the segment has a joint
            if(chain.getSegment(i).getJoint().getType()!=Joint::None){
		jac_tmp.setColumn(k++,t_tmp);
                j++;
            }

            T_tmp = total;
        }
	jac.data.noalias()=jac_tmp.data*chain.cm;
        return 0;
    }
}

