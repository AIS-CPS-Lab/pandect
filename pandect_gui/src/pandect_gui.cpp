#include <functional>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <threepp/threepp.hpp>  

#include <pandect_gui/pandect_gui.hpp>


namespace pandect_gui {


PandectGui::PandectGui() : Node("pandect_gui") {

  this->declareAndLoadParameter("param", param_, "TODO", true, false, false, 0.0, 10.0, 1.0);
  this->setup();
}


template <typename T>
void PandectGui::declareAndLoadParameter(const std::string& name,
														 T& param,
														 const std::string& description,
														 const bool add_to_auto_reconfigurable_params,
														 const bool is_required,
														 const bool read_only,
														 const std::optional<double>& from_value,
														 const std::optional<double>& to_value,
														 const std::optional<double>& step_value,
														 const std::string& additional_constraints) {

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto type = rclcpp::ParameterValue(param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
	if constexpr(std::is_integral_v<T>) {
	  rcl_interfaces::msg::IntegerRange range;
	  T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1);
	  range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value())).set__step(step);
	  param_desc.integer_range = {range};
	} else if constexpr(std::is_floating_point_v<T>) {
	  rcl_interfaces::msg::FloatingPointRange range;
	  T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1.0);
	  range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value())).set__step(step);
	  param_desc.floating_point_range = {range};
	} else {
	  RCLCPP_WARN(this->get_logger(), "Parameter type of parameter '%s' does not support specifying a range", name.c_str());
	}
  }

  this->declare_parameter(name, type, param_desc);

  try {
	param = this->get_parameter(name).get_value<T>();
	std::stringstream ss;
	ss << "Loaded parameter '" << name << "': ";
	if constexpr(is_vector_v<T>) {
	  ss << "[";
	  for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
	  ss << "]";
	} else {
	  ss << param;
	}
	RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
	if (is_required) {
	  RCLCPP_FATAL_STREAM(this->get_logger(), "Missing required parameter '" << name << "', exiting");
	  exit(EXIT_FAILURE);
	} else {
	  std::stringstream ss;
	  ss << "Missing parameter '" << name << "', using default value: ";
	  if constexpr(is_vector_v<T>) {
		ss << "[";
		for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
		ss << "]";
	  } else {
		ss << param;
	  }
	  RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
	  this->set_parameters({rclcpp::Parameter(name, rclcpp::ParameterValue(param))});
	}
  }

  if (add_to_auto_reconfigurable_params) {
	std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) {
	  param = p.get_value<T>();
	};
	auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}


rcl_interfaces::msg::SetParametersResult PandectGui::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {

  for (const auto& param : parameters) {
	for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
	  if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
		std::get<1>(auto_reconfigurable_param)(param);
		RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s'", param.get_name().c_str());
		break;
	  }
	}
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}


void PandectGui::setup() {

  // callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(std::bind(&PandectGui::parametersCallback, this, std::placeholders::_1));

  // subscriber for handling incoming messages
  subscriber_ = this->create_subscription<std_msgs::msg::Int32>("~/input", 10, std::bind(&PandectGui::topicCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscriber_->get_topic_name());
}


void PandectGui::topicCallback(const std_msgs::msg::Int32::ConstSharedPtr& msg) {

  RCLCPP_INFO(this->get_logger(), "Message received: '%d'", msg->data);
}


}


int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<pandect_gui::PandectGui>();
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(node);

	std::thread ros_thread(&rclcpp::executors::SingleThreadedExecutor::spin, &executor);
	ros_thread.detach();

	threepp::Canvas canvas;
	threepp::GLRenderer renderer(canvas.size());
	auto scene = threepp::Scene::create();
	auto camera = threepp::PerspectiveCamera::create(60, canvas.aspect(), 0.00001f, 1000.0f);
	camera->position.z = 1;

	threepp::OrbitControls controls(*camera, canvas);

	std::pair depth_dim = {320, 240};
	const int numParticles = 10000;
	auto mat = threepp::MeshBasicMaterial::create();
	mat->side = threepp::Side::Double;
	auto im = threepp::InstancedMesh::create(
		threepp::BoxGeometry::create(0.005, 0.005, 0.005),mat, numParticles);
	im->rotateZ(threepp::math::degToRad(180));
	im->rotateY(threepp::math::degToRad(180));

	scene->add(im);

	canvas.onWindowResize([&](threepp::WindowSize size) {
		camera->aspect = size.aspect();
		camera->updateProjectionMatrix();
		renderer.setSize(size);
	});

	threepp::Matrix4 matrix;
	canvas.animate([&]() {
		// Render loop
		for (int i = 0; i < numParticles; ++i) {
			matrix.identity();

		}
		renderer.render(*scene, *camera); 
		});
	std::cout << "Shutting down..." << std::endl;
	rclcpp::shutdown();

	return 0;
}
