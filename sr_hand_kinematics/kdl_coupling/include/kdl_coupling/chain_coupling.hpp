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

#ifndef KDL_CHAIN_COUPLING_HPP
#define KDL_CHAIN_COUPLING_HPP

#include <kdl/segment.hpp>
#include <string>
#include <Eigen/Core>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>

namespace KDL {
    typedef Eigen::MatrixXd (*updateFuncPtr)(const JntArray& q);  // Pointer to update function of coupling matrix
    /**
	  * \brief This class encapsulates a <strong>serial</strong> kinematic
	  * interconnection structure. It is build out of segments.
     *
     * @ingroup KinematicFamily
     */
    class Chain_coupling:public KDL::Chain {
    private:
	unsigned int nrOfIndJoints;
	updateFuncPtr updateFunction;
    public:
        /**
         * The constructor of a Chain_coupling, a new Chain_coupling is always empty.
         *
         */
        Chain_coupling();
        Chain_coupling(const Chain_coupling& in);
	Chain_coupling(const Chain& in);
        Chain_coupling& operator= (const Chain_coupling& in);
	Chain_coupling& operator= (const Chain& in);
	/**
	 * Determines if there is coupling between the joints of a Chain_coupling.
	 * @return true or false if there is or not coupling.
	 */
	bool isCoupled() const;
        /**
         * Gets the function which updates the value of the coupling matrix depending
         * on the current joint values.
         * @return pointer to update function.
	 */
        updateFuncPtr getUpdateCouplingFunction() const;
        /**
         * Sets the function which updates the value of the coupling matrix depending
         * on the current joint values.
         * @param updateFunc pointer to the function which implements the coupling update.
	 * @return true if the fucntion generates correct coupling matrices and false otherwise
	 */
        bool setUpdateCouplingFunction(updateFuncPtr updateFunc);
        /**
         * Calls the update function which changes the coupling matrix according to the
         * joint values passed as parameter.
         * @param q array of current joint values.
         */
	void updateCoupling(const JntArray& q);     
        /**
	 * Request the total number of independent joints in the Chain_coupling.
	 * It's the Coupling matrix rank.
	 * @return total nr of independ joints
	 */
        unsigned int getNrOfIndJoints()const {return nrOfIndJoints;};
        ~Chain_coupling();

	  // Matrix with coupling coefficients between joints
        Eigen::MatrixXd cm;
    };



// end of namespace KDL

#endif
