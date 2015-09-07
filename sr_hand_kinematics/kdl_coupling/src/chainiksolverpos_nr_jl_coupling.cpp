// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Copyright  (C)  2008  Mikael Mayer
// Copyright  (C)  2008  Julia Jesse

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

#include <kdl_coupling/chainiksolverpos_nr_jl_coupling.hpp>
#include <kdl_coupling/chainiksolvervel_wdls_coupling.hpp>
//#include <cstdio>

namespace KDL
{
    ChainIkSolverPos_NR_JL_coupling::ChainIkSolverPos_NR_JL_coupling(const Chain_coupling& _chain, const JntArray& _q_min, const JntArray& _q_max, ChainFkSolverPos& _fksolver,ChainIkSolverVel& _iksolver,
                                             unsigned int _maxiter, double _eps):
        chain(_chain), q_min(chain.getNrOfJoints()), q_max(chain.getNrOfJoints()), fksolver(_fksolver),iksolver(_iksolver),delta_q(_chain.getNrOfJoints()),
        maxiter(_maxiter),eps(_eps)
    {
    	q_min = _q_min;
    	q_max = _q_max;
    }

    int ChainIkSolverPos_NR_JL_coupling::CartToJnt(const JntArray& q_init, const Frame& p_in, JntArray& q_out)
    {
      q_out = q_init;
      Twist delta_twist_temp;
      unsigned int i;
      for(i=0;i<maxiter;i++)
      {
        fksolver.JntToCart(q_out,f);
        delta_twist = diff(f,p_in);
        delta_twist_temp= delta_twist;

        ChainIkSolverVel_wdls_coupling* iksolver_wdls;
        iksolver_wdls= dynamic_cast<ChainIkSolverVel_wdls_coupling*> (&iksolver);
        if(iksolver_wdls != NULL)  // Verify Mx matrix if ik velocity solver is based on WDLS
        {
          Eigen::MatrixXd Mx= iksolver_wdls->getWeightTS();
          // Remove the error of those components which are null in Mx matrix.
          // This is usually used to neglect the error of one or several components of the pose.
          // Thereby, a solution of 3D inverse kinematics (only position or only orientation) can be obtained.
          if(Mx(0,0) == 0.0) delta_twist_temp.vel.x(0.0);
          if(Mx(1,1) == 0.0) delta_twist_temp.vel.y(0.0);
          if(Mx(2,2) == 0.0) delta_twist_temp.vel.z(0.0);
          if(Mx(3,3) == 0.0) delta_twist_temp.rot.x(0.0);
          if(Mx(4,4) == 0.0) delta_twist_temp.rot.y(0.0);
          if(Mx(5,5) == 0.0) delta_twist_temp.rot.z(0.0);		
        }

        if(Equal(delta_twist_temp,Twist::Zero(),eps))
          break;

        iksolver.CartToJnt(q_out,delta_twist,delta_q);
          Add(q_out,delta_q,q_out);
//			printf("it:%d,dtw %f %f %f , dq:%f %f %f %f, q:%f %f %f %f\n",i,delta_twist.vel.x(),delta_twist.vel.y(),delta_twist.vel.z(),delta_q(0),delta_q(1),delta_q(2),delta_q(3),q_out(0),q_out(1),q_out(2),q_out(3));
          for(unsigned int j=0; j<q_min.rows(); j++) {
            if(q_out(j) < q_min(j))
              q_out(j) = q_min(j);
          }


          for(unsigned int j=0; j<q_max.rows(); j++) {
              if(q_out(j) > q_max(j))
                q_out(j) = q_max(j);
          }
//					 printf("it:%d,dtw %f %f %f , dq:%f %f %f %f, q:%f %f %f %f\n",i,delta_twist.vel.x(),delta_twist.vel.y(),delta_twist.vel.z(),delta_q(0),delta_q(1),delta_q(2),delta_q(3),q_out(0),q_out(1),q_out(2),q_out(3));
      }

        if(i!=maxiter)
        {
            return 0;
        }
        else
        {
            return -3;
        }
    }

    ChainIkSolverPos_NR_JL_coupling::~ChainIkSolverPos_NR_JL_coupling()
    {
    }

}

