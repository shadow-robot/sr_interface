Copyright UPMC 2012

This package contains KDL functions augmented with coupling capabilities.
It can be used in any IK module that requires coupling capable FK/IK. It is based on an URDF chain.

It should not be Shadow hand specific as the coupling coefficient can be set to anything from an outer function call
(for example hand_kinematics uses it as well as cartesian controller, setting the coupling coefficient accordingly)

Usage :
------

	* Init the KDL chain

KDL::Chain_coupling from this package inherits standard KCL::Chain and so can be filled correctly by standard functions
as long as you cast KDL::Chain_coupling to KDL:Chain before. Make sure you correctly fill additionnal parts of
KDL::Chain_coupling to your needs (coupling function etc...)

look at robot_mechanism_controllers for the cartesian controller to see how to fill a KDL chain from an URDF chain

ex :
  KDL::Chain_coupling kdl_chain_;
  KDL::Tree kdl_tree;
  kdl_parser::treeFromUrdfModel(robot_state->model_->robot_model_,kdl_tree);
  KDL::Chain kdl_chain_base= (KDL::Chain &)kdl_chain_; //cast to parent class
  kdl_tree.getChain(root_name_,tip_name,kdl_chain_base);
  chain_.init(robot_state_, root_name_, tip_name); //chain_ is a pr2_mechanism chain
  chain_.toKDL(kdl_chain_base);

   // Initialize coupling information in KDL chain
  kdl_chain_.setUpdateCouplingFunction(updateCouplingFF);
  
	* Updating the coupling

Before each IK/FK call, you should call the update coupling function (so variable coupling is possible)

best is to create a coupling function returning an Eigen matrix 

ex :
Eigen::MatrixXd updateCouplingFF(const KDL::JntArray& q)
{
  Eigen::MatrixXd cm(4,3); // must be the correct size
  //fill the matrix here
  return cm;
}




tutorial later to come.

