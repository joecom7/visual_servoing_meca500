#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "meca500_interfaces/srv/get_image_jacobian.hpp"

using GetImageJacobian = meca500_interfaces::srv::GetImageJacobian;

const double DEFAULT_HORIZONTAL_FOV_DEGREES = 85.2;
const double DEFAULT_IMAGE_WIDTH_PIXELS = 1280;

class ImageJacobianNode : public rclcpp::Node
{
public:
  ImageJacobianNode() : Node("image_jacobian_node")
  {
    // Dichiarazione dei parametri e lettura immediata
    this->declare_parameter<double>("horizontal_fov_degrees", DEFAULT_HORIZONTAL_FOV_DEGREES);
    this->declare_parameter<int>("image_width_pixels", DEFAULT_IMAGE_WIDTH_PIXELS);

    double horizontal_fov = this->get_parameter("horizontal_fov_degrees").as_double() * M_PI / 180.0;
    int image_width_pixels = this->get_parameter("image_width_pixels").as_int();
    f_ = image_width_pixels / (2.0 * tan(horizontal_fov / 2.0));

    service_ = this->create_service<GetImageJacobian>(
        "get_matrix", [this](const std::shared_ptr<GetImageJacobian::Request> request,
                             std::shared_ptr<GetImageJacobian::Response> response) {
          Eigen::Matrix<double, 2, 6> L;
          // Riga 1
          L(0, 0) = -f_ / (rho_u * request->depth);
          L(0, 1) = 0.0;
          L(0, 2) = request->u / request->depth;
          L(0, 3) = (rho_u * request->u * request->v) / f_;
          L(0, 4) = -(f_ * f_ + rho_u * rho_u * request->u * request->u) / (rho_u * f_);
          L(0, 5) = request->v;
          // Riga 2
          L(1, 0) = 0.0;
          L(1, 1) = -f_ / (rho_v * request->depth);
          L(1, 2) = request->v / request->depth;
          L(1, 3) = (f_ * f_ + rho_v * rho_v * request->v * request->v) / (rho_v * f_);
          L(1, 4) = -(rho_v * request->u * request->v) / f_;
          L(1, 5) = -request->u;

          response->rows = L.rows();
          response->cols = L.cols();
          response->image_jacobian.resize(L.size());
          Eigen::Map<Eigen::VectorXd>(response->image_jacobian.data(), L.size()) =
              Eigen::Map<const Eigen::VectorXd>(L.data(), L.size());
        });

    RCLCPP_INFO(this->get_logger(), "Image Jacobian service node ready!");
  }

private:
  rclcpp::Service<GetImageJacobian>::SharedPtr service_;
  double f_;
  const double rho_u = 1.0;
  const double rho_v = 1.0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageJacobianNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
