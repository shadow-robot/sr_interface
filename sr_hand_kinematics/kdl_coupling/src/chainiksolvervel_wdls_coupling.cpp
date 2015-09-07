// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// Modified by Juan A. Corrales, ISIR, UPMC
// Added coupled joint support based on a modified version of the patch by Federico Ruiz: 
// http://www.orocos.org/forum/rtt/rtt-dev/patches-coupled-joints-locked-joints-and-python-executable-detection

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

#include <kdl_coupling/chainiksolvervel_wdls_coupling.hpp>
#include <kdl/utilities/svd_eigen_HH.hpp>
//#include <cstdio>
namespace KDL
{
    
    ChainIkSolverVel_wdls_coupling::ChainIkSolverVel_wdls_coupling(const Chain_coupling& _chain,double _eps,int _maxiter):
        chain(_chain),
        jnt2jac(chain),
        jac(chain.getNrOfIndJoints()),
        U(MatrixXd::Zero(6,chain.getNrOfIndJoints())),
        S(VectorXd::Zero(chain.getNrOfIndJoints())),
        V(MatrixXd::Zero(chain.getNrOfIndJoints(),chain.getNrOfIndJoints())),
        eps(_eps),
        maxiter(_maxiter),
        tmp(VectorXd::Zero(chain.getNrOfIndJoints())),
        tmp_jac(MatrixXd::Zero(6,chain.getNrOfJoints())),
        tmp_jac_weight1(MatrixXd::Zero(6,chain.getNrOfIndJoints())),
        tmp_jac_weight2(MatrixXd::Zero(6,chain.getNrOfIndJoints())),
        tmp_ts(MatrixXd::Zero(6,6)),
        tmp_js(MatrixXd::Zero(chain.getNrOfIndJoints(),chain.getNrOfIndJoints())),
        weight_ts(MatrixXd::Identity(6,6)),
        weight_js(MatrixXd::Identity(chain.getNrOfIndJoints(),chain.getNrOfIndJoints())),
        lambda(0.0)
    {
    }
    
    ChainIkSolverVel_wdls_coupling::~ChainIkSolverVel_wdls_coupling()
    {
    }
    
    void ChainIkSolverVel_wdls_coupling::setWeightJS(const MatrixXd& Mq){
        weight_js = Mq;
    }
    MatrixXd ChainIkSolverVel_wdls_coupling::getWeightJS(){
        return weight_js;
    }
    
    void ChainIkSolverVel_wdls_coupling::setWeightTS(const MatrixXd& Mx){
        weight_ts = Mx;
    }
    MatrixXd ChainIkSolverVel_wdls_coupling::getWeightTS()
    {
	return weight_ts;
    }

    void ChainIkSolverVel_wdls_coupling::setLambda(const double& lambda_in)
    {
        lambda=lambda_in;
    }
    
    int ChainIkSolverVel_wdls_coupling::CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
    {
	  // Update the coupling matrix of the chain
	this->chain.updateCoupling(q_in);
	  // Compute the Jacobian
        jnt2jac.JntToJac(q_in,jac);
        
        double sum;
        unsigned int i,j;
 
        // Create the Weighted jacobian
       // tmp_jac_weight1 = jac.data.lazyProduct(weight_js);  // RE-USE THIS LINE IF FAILURE
       // tmp_jac_weight2 = weight_ts.lazyProduct(tmp_jac_weight1);  // RE-USE THIS LINE IF FAILURE
        tmp_jac_weight1.noalias() = jac.data*weight_js;
        tmp_jac_weight2.noalias() = weight_ts*tmp_jac_weight1;

        // Compute the SVD of the weighted jacobian
        int ret = svd_eigen_HH(tmp_jac_weight2,U,S,V,tmp,maxiter);
       // Pre-multiply U and V by the task space and joint space weighting matrix respectively
        
        Eigen::MatrixXd U_temp(6,6);
        U_temp.setZero();

        if(U.cols() >= 6)
                U_temp= U.topLeftCorner(6,6);
        else
                U_temp.topLeftCorner(U.rows(),U.cols())=U;

       // tmp_ts = weight_ts.lazyProduct(U_temp);  // RE-USE THIS LINE IF FAILURE
        tmp_js = weight_js.lazyProduct(V);  // DO NOT REMOVE LAZY HERE
        tmp_ts.noalias() = weight_ts*U_temp;
        
        // tmp = (Si*U'*Ly*y), 
        for (i=0;i<jac.columns();i++) {
            sum = 0.0;
            for (j=0;j<jac.rows();j++) {
                if(i<6)
                    sum+= tmp_ts(j,i)*v_in(j);
                else
                    sum+=0.0;
            }
            if(S(i)==0||S(i)<eps)
                tmp(i) = sum*((S(i)/(S(i)*S(i)+lambda*lambda)));   
            else
                tmp(i) = sum/S(i);
        }

       // qdot_out.data=(chain.cm*(tmp_js*tmp).lazy()).lazy();
        qdot_out.data.noalias()=chain.cm*(tmp_js*tmp);
        return ret;
    }
    
}
