/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#ifndef _SDK_NODE_H
#define _SDK_NODE_H

#include <string>
#include <vector>

#include <ros/ros.h>

#include <sdk_interface.h>
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "nepi_ros_interfaces/Reset.h"

namespace Numurus
{

/**
 * @brief      Base class to represent a ROS node in the Numurus SDK
 *
 * 			   This class encapsulates the components required for a ROS node, including both a public namespace and private namespace NodeHandle, and
 * 			   vectors of subscribers, servicers, etc. for ROS operations.
 *
 */
class SDKNode
{
public:
	// Make the SDKInterface constructor a friend so that it can push itself onto the sdk_interfaces vector.
	friend SDKInterface::SDKInterface(SDKNode *parent, ros::NodeHandle *parent_pub_nh, ros::NodeHandle *parent_priv_nh);

class NodeParamBase
{
public:
	NodeParamBase(std::string param_name, SDKNode *parent):
		_param_name{param_name},
		_parent{parent}
	{
		_parent->_param_list.push_back(this);
	}

	const std::string getName() const {return _param_name;}
	void clearRetrievalFlag(){_retrieval_flag = false;}
	bool getRetrievalFlag() const {return _retrieval_flag;}

protected:
	const std::string _param_name;
	SDKNode *_parent; // non-const to allow for _param_list.insert()
	bool _retrieval_flag = false;
};

/**
 * @brief      Class to represent node parameters
 *
 *			   This class is useful primarily to provide a smart assignment operator that will respect the config_rt setting.
 *			   It also provides some utility in its encapsulation of the param name, helping to avoid a lot of strings peppered
 *			   through the code.
 *
 *				 It would be helpful if this class's non-templated based class defined pure virtual functions like retrieve() so
 *				 SDKNode::retrieveParams() could automatically iterate over this list, but seems problematic with templatization
 *				 in the concrete instances... perhaps an experiment for another day!
 *
 * @tparam     T     Parameter type. Must be a valid ROS param type.
 */

template <class T>
class NodeParam : public NodeParamBase
{
public:
	NodeParam(std::string param_name, T default_val, SDKNode *parent):
		NodeParamBase(param_name, parent),
		_param_data{default_val}
	{
		// Retrieve automatically to establish parameter in the param server
		//retrieve(); // Don't do this anymore as it will interfere with the warnUnretrievedParams() system for SDKInterfaces
	}
	
	/**
	 * @brief      Conversion operator to allow a NodeParam to be implicitly converted to its template type
	 */
	operator T() {return _param_data;}
	operator T() const {const T data = _param_data; return data;}

	/**
	 * @brief      Retrieves a parameter from the param server
	 *
	 * 			   If param server has no value for requested param, this will establish it in the param server from value of param_storage.
	 *
	 * @return     true if parameter was found on server, false otherwise (but param thereafter exists on server)
	 */
	bool retrieve()
	{
		// Always set the retrieval flag so that parent node can tell which params have been retrieved (since last retrieval flag clear)
		_retrieval_flag = true;
		if (true == _parent->n_priv.getParam(_param_name, _param_data))
		{
			ROS_INFO("%s: Updating %s from param server", _parent->getUnqualifiedName().c_str(), _param_name.c_str());
			return true;
		}
		else
		{
			ROS_WARN("%s: unable to init %s from param server, using existing val instead", _parent->getUnqualifiedName().c_str(), _param_name.c_str());
			// And attempt write it back so that the config file has something for next time
			_parent->n_priv.setParam(_param_name, _param_data);
		}
		return false;
	}

	/**
	 * @brief      Assignment operator to allow assignment from an instance of the template type
	 *
	 * @param[in]  rhs   The right hand side
	 *
	 * @return     Reference to this NodeParam instance
	 */
	virtual NodeParam& operator=(const T& rhs)
	{
		_param_data = rhs;
		_parent->n_priv.setParam(_param_name, _param_data);
		if (true == _parent->_save_cfg_rt)
		{
			std_msgs::String node_name;
			node_name.data = _parent->getQualifiedName();
			_parent->_store_params_pub.publish(node_name);
		}
		return *this;
	}

	virtual NodeParam& operator=(const NodeParam& rhs)
	{
		return (*this = rhs._param_data); // Use the other assignment operator
	}

	bool operator!=(const T& rhs) const
	{
		return (_param_data != rhs);
	}

	bool operator==(const T& rhs) const
	{
		return (_param_data == rhs);
	}

private:
	T _param_data;
}; // class NodeParam



	/**
	 * @brief      Constructs the object.
	 *
	 */
	SDKNode();
	~SDKNode();

	/**
	 * @brief      Initialize the node
	 *
	 * 			   This method calls initPublishers(), retrieveParams(), initSubscribers, and initServices()
	 * 			   (in that order) to completely initialize the ROS components of the node. If not called explicitly
	 * 			   (and it should be), run() will call this method
	 *
	 * @note       Subclasses may override this method but should call back to this base implementation or otherwise replicate its behavior
	 */
	virtual void init();

	/**
	 * @brief      Execute the main work method
	 *
	 * 			   This base implementation simply calls ros::spin(). It will call init() if it hasn't already been called
	 * 			   to preserve a previous behavior, but users should call init() explicitly before run() to control when the
	 * 			   initializations occur.
	 *
	 * @note       Subclasses may override this method but should preserve the conditional init() and some version of ros::spin()
	 */
	virtual void run();

	/**
	 * @brief      Get the name of the SDK Node including fully qualified namespace.
	 *
	 * @return     The fully qualifed (i.e., namespaced) name.
	 */
	inline const std::string getQualifiedName() const {return ros::this_node::getName();}

	/**
	 * @brief      Returns the node name as determined at SDKNode constructor time
	 *
	 * @return     The simple name (no namespace).
	 */
	inline const std::string getUnqualifiedName() const {return ns_tokens[NODE_NAME_INDEX];}

	/**
	 * @brief      Gets the display name.
	 *
	 * @return     The display name.
	 */
	inline std::string getDisplayName() {return _display_name;}

	/**
	 * @brief      Return the public namespace of the SDKNode
	 *
	 * @return     The public namespace.
	 */
	inline const std::string getPublicNamespace() const {return n.getNamespace();}

	/**
	 * @brief      Return the private namespace of the SDKNode
	 *
	 * @return     The private namespace.
	 */
	inline const std::string getPrivateNamespace() const {return n_priv.getNamespace();}

protected:
	const std::vector<std::string> ns_tokens;
	static constexpr auto ROOTNAME_INDEX = 1;
	static constexpr auto DEV_TYPE_INDEX = 2;
	static constexpr auto DEV_SN_INDEX = 3;
	const size_t NODE_NAME_INDEX; // Will be the last token, whichever that one is

  double min_rate_hz = 0.25;
  double max_rate_hz = 40;
  double current_rate_hz;

	/**
	 * The public namespace node handle. This is used for any ROS primitives that should resolve into the top-level namespace for this node. In particular, subscribing to
	 * general topics and services occurs through this node handle.
	 */
	ros::NodeHandle n;

	/**
	 * The private namespace node handle. This is used for any ROS primitives that must resolve into a subnamespace of this node to avoid conflicts. In particular, param access
	 * use this node handle.
	 */
	ros::NodeHandle n_priv;

	/**
	 * The list of parameters for retrieval checking, etc. Must come BEFORE any params below to ensure it is constructed first
	 */
	std::vector<NodeParamBase *> _param_list;

	/**
	 * Display name of the node. Could be modified by users (though no interface to do so for this generic base class)
	 */
	NodeParam<std::string> _display_name;

	/**
	 * Vector of return handles from NodeHandle::advertiseService.
	 * These handles must have a lifetime as long as the NodeHandle, so are best appropriated to a member variable container
	 */
	std::vector<ros::ServiceServer> servicers;

	/**
	 * Vector of return handles from NodeHandle::subscribe.
	 * These handles must have a lifetime as long as the NodeHandle, so are best appropriated to a member variable container
	 */
	std::vector<ros::Subscriber> subscribers;

	/**
	 * Vector of SDKInterface pointers to allow generic iteration over all interfaces for setup (initSubscribers, etc.)
	 */
	std::vector<SDKInterface *> sdk_interfaces;

	bool _initialized = false;

	/**
	 * @brief      Determine whether the namespace is valid
	 *
	 * @return     True if valid, false otherwise
	 */
	virtual inline bool validateNamespace(){return ns_tokens.size() > 3;}

	/**
	 * @brief      Advertise topics to be published
	 *
	 * @note       {Subclasses may override this method, but should call back to the base
	 * 				class method to ensure the generic initialization proceeds.}
	 */
	virtual void initPublishers();

	/**
	 * @brief      Retrieve all ROS parameters from param server.
	 *
	 * 			   The ROS parameter server launches with parameters from the various config. files.
	 * 			   SDKNodes never interact with these files directly, rather gathering params from the
	 * 			   param server, and leveraging the config_mgr node to save and restore params to/from
	 * 			   the config files.
	 *
	 * @note       {Subclasses may override this method to do specific param initialization, but should call SDKNode::retrieveParams() wihin the overriden implementation
	 * 				to ensure the generic initialization proceeds.}
	 */
	virtual void retrieveParams();

	/**
	 * @brief      Initialize subscribers and add to the list of subscriber handles
	 *
	 * @note       {Subclasses may override this method, but should call back to the base
	 * 				class method to ensure the generic initialization proceeds.}
	 */
	virtual void initSubscribers();

	/**
	 * @brief      Advertise services to be provided
	 *
	 * @note       {Subclasses may override this method, but should call back to the base
	 * 				class method to ensure the generic initialization proceeds.}
	 */
	virtual void initServices();

	virtual void initServiceClients();

	void warnUnretrievedParams();

	/**
	 * @brief      Handle a request to save the current ROS configuration
	 *
	 * @param[in]  empty    Just a ROS-required placeholder
	 */
	void saveCfgHandler(const std_msgs::Empty::ConstPtr &empty);

	void saveCfgRtHandler(const std_msgs::Bool::ConstPtr &msg);

	/**
	 * @brief      Handle a request to reset the node
	 *
	 * 			   The request can be one of a number of reset types. This methods simply calls into
	 * 			   reset type-specific functions that may be overridden by subclasses
	 *
	 * @param[in]  msg   The message containing the reset type requested
	 */
	void resetHandler(const nepi_ros_interfaces::Reset::ConstPtr &msg);

  /**
	 * @brief      Handle a request to set the node's throttle value
	 *
	 * 			   This assigns a rate between min_rate_hz and max_rate_hz
	 *
	 * @param[in]  msg   The message containing the throttle ration [0.0 - 1.0]
	 */
	void applyThrottleHandler(const std_msgs::Float32::ConstPtr &msg);

	/**
	 * @brief      Execute a user reset
	 *
	 * 			   This generic implementation simply restores parameters to their param server values
	 * 			   which are nominally the same as the config file values that were loaded at start-up time.
	 *
	 * 			   Subclasses do not typically need to override this method
	 */
	virtual void userReset();

	/**
	 * @brief      Exectue a factory reset, restoring configuration to it's factory default
	 *
	 * 			   Subclasses do not typically need to override this method
	 */
	virtual void factoryReset();

	/**
	 * @brief      Execute a software reset, terminating the node and allowing the ROS launch system to restart it.
	 *
	 * 			   Subclasses do not typically need  to override this method.
	 */
	virtual void softwareReset();

	/**
	 * @brief      Execute a hardware reset.
	 *
	 * 			   This implementation of this reset mode is typically dependent on the underlying
	 * 			   hardware proxied by the node, so subclasses will typically override this method.
	 *
	 * 			   The generic implementation included here simply calls softwareReset()
	 */
	virtual void hardwareReset();

	/**
	 * @brief      Save the node's configuration file
	 *
	 * 			   This method serves to coordinate a save operation with the parameter server by
	 * 			   informing the param server that it will be saving, updating all parameters (via updateParams), then
	 * 			   informing the param server that it is done. This allows the param server to determine that a
	 * 			   set of nodes is attempting to save, then delay the actual file saving until they have finished updates.
	 */
	void saveCfg();

  /**
   * @brief    Set the throttle ratio for the node
   *
   *        The throttle ratio controls the rate that the node executes. A value
   *        of 1.0 means run at the max_rate and a value of 0.0 means run at the min
   *        rate, linearly scaling between the two
   */
  void setThrottleRatio(float throttle_ratio);

	/**
   * @brief    Submit an error message for inclusion in next sys_status
   *
   */
	void submitSysErrorMsg(const std::string &error_msg);

private:
	ros::Publisher _store_params_pub;
	ros::Publisher _submit_sys_err_msg_pub;
	bool _save_cfg_rt = false;
};

} // namespace Numurus

#endif //_SDK_NODE_H
